"""Commit calibration outputs and push the current branch. Failures are returned,
not raised, so a missing remote/credentials never crashes a calibration."""
import subprocess


def _run(cmd, cwd):
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError((r.stdout or "") + (r.stderr or ""))
    return r.stdout


def commit_and_push(paths, message, cwd=None):
    """git add <paths> && commit && push origin HEAD. Returns (ok, output)."""
    try:
        _run(["git", "add", *paths], cwd)
        _run(["git", "commit", "-m", message], cwd)
        out = _run(["git", "push", "origin", "HEAD"], cwd)
        return True, out
    except RuntimeError as e:
        return False, str(e)
