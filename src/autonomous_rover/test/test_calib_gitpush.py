from autonomous_rover.nodes.master.calibration import gitpush


class _FakeRun:
    def __init__(self):
        self.calls = []

    def __call__(self, cmd, cwd=None, capture_output=True, text=True):
        self.calls.append(cmd)
        class R:
            returncode = 0
            stdout = "ok"
            stderr = ""
        return R()


def test_commit_and_push_runs_add_commit_push(monkeypatch):
    fake = _FakeRun()
    monkeypatch.setattr(gitpush.subprocess, "run", fake)
    ok, out = gitpush.commit_and_push(["a.yaml", "b.yaml"], "msg", cwd="/repo")
    assert ok is True
    assert fake.calls[0] == ["git", "add", "a.yaml", "b.yaml"]
    assert fake.calls[1] == ["git", "commit", "-m", "msg"]
    assert fake.calls[2] == ["git", "push", "origin", "HEAD"]


def test_commit_and_push_reports_failure(monkeypatch):
    def failing(cmd, cwd=None, capture_output=True, text=True):
        class R:
            returncode = 1
            stdout = ""
            stderr = "auth failed"
        return R()
    monkeypatch.setattr(gitpush.subprocess, "run", failing)
    ok, out = gitpush.commit_and_push(["a.yaml"], "msg", cwd="/repo")
    assert ok is False
    assert "auth failed" in out
