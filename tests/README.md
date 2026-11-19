# Offline verification

From the repository root after installing `.[test]`:

```bash
python -B -m pytest -q -p no:cacheprovider tests
```

| File | Coverage / when to update |
| --- | --- |
| test_core.py | Protocol packing, model scaling, session states, transport ownership, faults |
| test_pipeline.py | Sequential startup, asynchronous control, TX confirmation, timing policies |
| test_receive.py | RX/state-lock isolation, priority inbox, missing/late echoes and overflow |
| test_setup_tool.py | Mocked platform setup, readiness checks and recovery refusal |
| test_signed_limits.py | Signed angle bounds and command/feedback validation |
| test_streaming.py | Latest target groups, partial submissions, watchdogs and stale feedback |
| test_zero_recovery.py | Direct signed Go Zero recovery from outside joint limits |

Scripted transports send no real CAN frames. Timing assertions test software
contracts, not hard real-time performance. GUI branch also has adapter/HTTP tests
in `motor_controller_beta_v2/tests`. CI runs the suite on Python 3.10–3.13.
