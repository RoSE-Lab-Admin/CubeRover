# DAQ

This code runs on the DAQ Teensey, which sends data to the lab computer.

## Symlink Info Info

The `src` folder is a symlink pointing to the `Arduino_ROS/src` folder.

**Recreating the Symbolic Link.** If you ever need to recreate the symlink:

1. Navigate to the `PySerial/DAQ/` folder.
2. Create the symlink: `ln -s ../../../Arduino_ROS/src src`

## Windows Users - Symlink Setup

Windows users will need to enable symbolic links for this to work correctly.

Enable Symbolic Links Globally
```bash
git config --global core.symlinks true
```

### Enable Symlinks in Git
You can enable symlink support globally or for a specific repository:

**Global configuration:**
```bash
git config --global core.symlinks true
```

**One-time clone (recommended):**
To ensure links are created correctly from the start, use the `-c` flag during cloning while running the terminal as an Administrator:
```bash
git clone -c core.symlinks=true <repository-url>
```

**Updating an existing repo:**
If you have already pulled the repo and see text files instead of links:
- Enable the setting: `git config core.symlinks true`
- Refresh the working tree: `git reset --hard HEAD` (Note: This will discard uncommitted changes).


### Configure Windows Permissions

Windows requires special privileges to create symbolic links. You must fulfill one of the following: 

- **Option 1: Run as Administrator.** Always run your terminal (Git Bash, Command Prompt, or PowerShell) as an Administrator when pulling or resetting.
- **Option 2: Enable Developer Mode.** Go to Settings > Update & Security > For developers and toggle Developer Mode to On. This allows non-administrators to create symlinks.
- **Option 3: Local Group Policy.** If Developer Mode is not an option, you can grant the "Create symbolic links" right to your user account via gpedit.msc under Computer Configuration > Windows Settings > Security Settings > Local Policies > User Rights Assignment.Troubleshooting