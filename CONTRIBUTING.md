# Contributing

This is experimental motor-control software. Start with an issue describing the
problem, hardware, firmware and intended behavior. Use a fork/feature branch and
pull request; do not commit recordings, credentials or machine-specific settings.

## Development

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -B -m pytest -q -p no:cacheprovider tests
```

On the GUI branch also run `motor_controller_beta_v2/tests` in the same pytest
command. Run `bash -n` on changed shell scripts. Do not run setup, recovery,
enable, zero or movement against hardware as part of CI. See the
[source editing guide](src/handycan/README.md) and [test map](tests/README.md).

## Pull requests

Explain the trigger, before/after behavior, modified timing/limits and validation.
Add a regression test for behavioral fixes. Keep one socket owner, one RX reader,
ordered special commands and bounded queues. Do not hide errors by automatic
reset/reconnect, increase deadlines without evidence, or add implicit activation.
Document public API/configuration changes and preserve CLI help.

For a new motor model, identify exact firmware and cite its protocol document.
Test encoding endpoints, feedback identity, layout and invalid frames. Rated or
peak torque is not necessarily the MIT encoding range. Keep operating limits
separate from wire scaling.

Hardware test reports must distinguish bench observations from offline tests:
include board/OS/kernel, adapter/transceiver, motor profiles/IDs, bitrate, rate,
termination, timestamped interface counters and sanitized diagnostic evidence.
Obtain the hardware operator's approval before activation; support joints and
use an independent means of stopping hazardous motion.

Contribute only material you have the right to license. Contributions are under
the repository's [MIT license](LICENSE); retain third-party notices and disclose
copied/adapted material. Dependencies retain their own licenses.

Discuss technical issues respectfully. Do not publish private contact details,
access tokens or personal logs in issues or pull requests.
