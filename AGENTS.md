# Agent Development Guide

## Git

Do not make commits. The developer owns git, so unless specifically asked, do not make commits on their behalf.

## Formatting

After doing any work and before returning back to user, format the code using:

```bash
python .\taproot-scripts\clang_format_all.py dirs .\huskybot-mcb-project\src\ .\huskybot-mcb-project\test\
```

## Agent Skills

### Issue Tracker

Issues and specs live as Markdown files under `.scratch/`. See `docs/agents/issue-tracker.md`.

### Domain Docs

Single-context layout: `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.
