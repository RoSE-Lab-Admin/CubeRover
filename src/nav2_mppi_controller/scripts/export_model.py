#!/usr/bin/env python3
"""
Export a trained MLP dynamics model to TorchScript for use in the C++ MPPI controller.

The exported module:
  - Embeds the normalizer (mean, std) as registered buffers so C++ only loads one .pt file
  - forward(x: Tensor[B, L+H, 2]) -> Tensor[B, H, 3]
      x   : raw (unnormalized) command sequence  [vx_cmd, wz_cmd] per step
      out : residual corrections in robot frame  [dx, dy, dtheta] per step
            full prediction = kinematic_integration(x) + out

Usage:
  python3 export_model.py [--arch mlp_small] [--lookback 20] [--horizon 20]

Outputs:
  models/<arch>_scripted.pt   (TorchScript module, ready for torch::jit::load in C++)
"""

import argparse
import pickle
import sys
from pathlib import Path

import torch
import torch.nn as nn

# ── match the definitions in run_nn_dynamics_v3.py exactly ────────────────────
LOOKBACK = 20
HORIZON  = 20

ARCH_PARAMS = {
    "mlp_small":  {"hidden": 64,  "n_layers": 2},
    "mlp_medium": {"hidden": 128, "n_layers": 3},
    "mlp_large":  {"hidden": 256, "n_layers": 4},
}


class MLP(nn.Module):
    """Identical to the MLP in run_nn_dynamics_v3.py."""
    horizon: int  # explicit type annotation for TorchScript

    def __init__(self, lookback: int, horizon: int, hidden: int, n_layers: int):
        super().__init__()
        self.horizon = horizon
        in_dim = (lookback + horizon) * 2
        layers = [nn.Linear(in_dim, hidden), nn.LayerNorm(hidden), nn.ReLU()]
        for _ in range(n_layers - 1):
            layers += [nn.Linear(hidden, hidden), nn.LayerNorm(hidden), nn.ReLU()]
        layers.append(nn.Linear(hidden, horizon * 3))
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.net(x.flatten(1))
        return out.reshape(x.shape[0], self.horizon, 3)


class NormalizedMLP(nn.Module):
    """
    Wraps MLP with embedded normalizer so C++ doesn't need the separate pkl.

    Registered buffers (norm_mean, norm_std) are saved in the TorchScript module
    and move to whatever device the module lives on via .to(device).
    """
    norm_mean: torch.Tensor  # shape (2,)  [vx_mean, wz_mean]
    norm_std:  torch.Tensor  # shape (2,)  [vx_std,  wz_std]

    def __init__(self, mlp: MLP, norm_mean: torch.Tensor, norm_std: torch.Tensor):
        super().__init__()
        self.mlp = mlp
        self.register_buffer('norm_mean', norm_mean)
        self.register_buffer('norm_std',  norm_std)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, lookback+horizon, 2)  raw commands — [vx_cmd, wz_cmd] per step
        Returns:
            (B, horizon, 3)  robot-frame residuals — [dx, dy, dtheta] per future step
        """
        x_norm = (x - self.norm_mean) / self.norm_std
        return self.mlp(x_norm)


def export(arch: str, lookback: int, horizon: int, models_dir: Path):
    if arch not in ARCH_PARAMS:
        sys.exit(f"Unknown arch '{arch}'. Choices: {list(ARCH_PARAMS)}")

    weights_path = models_dir / f"{arch}.pt"
    norm_path    = models_dir / "normalizer.pkl"
    out_path     = models_dir / f"{arch}_scripted.pt"

    if not weights_path.exists():
        sys.exit(f"Weights not found: {weights_path}")
    if not norm_path.exists():
        sys.exit(f"Normalizer not found: {norm_path}")

    # Load normalizer
    with open(norm_path, 'rb') as f:
        norm = pickle.load(f)
    norm_mean = torch.tensor(norm['mean'], dtype=torch.float32)
    norm_std  = torch.tensor(norm['std'],  dtype=torch.float32)
    print(f"Normalizer — mean: {norm_mean.tolist()}, std: {norm_std.tolist()}")

    # Build and load MLP
    params = ARCH_PARAMS[arch]
    mlp = MLP(lookback=lookback, horizon=horizon, **params)
    mlp.load_state_dict(torch.load(weights_path, map_location='cpu'))
    mlp.eval()
    print(f"Loaded {arch}: {sum(p.numel() for p in mlp.parameters()):,} params")

    # Wrap with normalizer
    model = NormalizedMLP(mlp, norm_mean, norm_std)
    model.eval()

    # Verify forward pass before scripting
    dummy = torch.zeros(4, lookback + horizon, 2)
    with torch.no_grad():
        out = model(dummy)
    assert out.shape == (4, horizon, 3), f"Unexpected output shape: {out.shape}"
    print(f"Forward pass OK: {dummy.shape} → {out.shape}")

    # Export to TorchScript
    scripted = torch.jit.script(model)
    scripted.save(str(out_path))
    print(f"Saved TorchScript model → {out_path}")

    # Verify reload
    reloaded = torch.jit.load(str(out_path))
    reloaded.eval()
    with torch.no_grad():
        out2 = reloaded(dummy)
    assert torch.allclose(out, out2, atol=1e-6), "Reload verification failed"
    print("Reload verification passed.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Export NN dynamics model to TorchScript')
    parser.add_argument('--arch',     default='mlp_small', choices=list(ARCH_PARAMS))
    parser.add_argument('--lookback', default=LOOKBACK, type=int)
    parser.add_argument('--horizon',  default=HORIZON,  type=int)
    args = parser.parse_args()

    models_dir = Path(__file__).resolve().parent.parent / 'models'
    export(args.arch, args.lookback, args.horizon, models_dir)
