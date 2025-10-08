#!/usr/bin/env bash
# Git SSH Profile Manager
# Usage:
#   ./git-profile.sh list
#   ./git-profile.sh use <profile> [--dry-run]
#   ./git-profile.sh current

SSH_CONFIG="$HOME/.ssh/config"

# Extract available hosts from ssh config
parse_profiles() {
    grep -E '^\s*Host\s+' "$SSH_CONFIG" \
        | awk '{for (i=2; i<=NF; i++) print $i}' \
        | grep -v '\*'
}

list_profiles() {
    echo "Contents of $SSH_CONFIG:"
    echo "------------------------------------"
    cat "$SSH_CONFIG"
    echo "------------------------------------"
    echo "Pick a 'Host' entry as your profile (e.g. github-work, github-personal)."
}

current_profile() {
    if git rev-parse --is-inside-work-tree &>/dev/null; then
        remote_url=$(git remote get-url $(git remote | head -n1))
        if [[ "$remote_url" =~ git@([^:]+): ]]; then
            echo "Current profile: ${BASH_REMATCH[1]}"
        else
            echo "Could not detect current profile (URL: $remote_url)"
        fi
    else
        echo "Not inside a Git repo"
    fi
}

use_profile() {
    local profile="$1"
    local dry_run="$2"

    if [[ -z "$profile" ]]; then
        echo "Error: no profile specified"
        echo "Usage: $0 use <profile> [--dry-run]"
        exit 1
    fi

    # Validate profile against ssh config
    if ! parse_profiles | grep -qx "$profile"; then
        echo "Error: profile '$profile' not found in $SSH_CONFIG"
        echo "Available profiles are:"
        parse_profiles
        exit 1
    fi

    if git rev-parse --is-inside-work-tree &>/dev/null; then
        for remote in $(git remote); do
            old_url=$(git remote get-url "$remote")
            if [[ "$old_url" =~ git@([^:]+):(.*) ]]; then
                current_host="${BASH_REMATCH[1]}"
                repo_path="${BASH_REMATCH[2]}"
                new_url="git@$profile:$repo_path"

                if [[ "$current_host" == "$profile" ]]; then
                    echo "Remote '$remote' already set to profile '$profile' ($old_url)"
                else
                    if [[ "$dry_run" == "--dry-run" ]]; then
                        echo "[DRY RUN] Would update '$remote':"
                        echo "    old: $old_url"
                        echo "    new: $new_url"
                    else
                        git remote set-url "$remote" "$new_url"
                        echo "Updated remote '$remote' to: $new_url"
                    fi
                fi
            else
                echo "Skipping remote '$remote' (not SSH format): $old_url"
            fi
        done
    else
        echo "Not inside a Git repo, nothing to update"
    fi
}

case "$1" in
    list) list_profiles ;;
    use)  use_profile "$2" "$3" ;;
    current) current_profile ;;
    *)
        echo "Usage: $0 [list|use <profile> [--dry-run]|current]"
        exit 1
        ;;
esac
