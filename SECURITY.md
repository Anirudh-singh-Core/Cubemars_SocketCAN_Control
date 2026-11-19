# Security and safety reports

This beta is not a certified industrial safety controller. Read [SAFETY.md](SAFETY.md).
There is no guaranteed support lifetime or security-response SLA.

Do not post credentials or exploit details publicly. If GitHub's **Security →
Report a vulnerability** button is available, use its private reporting flow.
Private reporting availability depends on repository settings. If it is absent,
open a minimal issue asking the maintainer to establish a private reporting
channel, without including sensitive details or reproduction payloads.

Include the affected commit, platform, impact and a minimal reproduction using
an offline transport where possible. A CAN fault report should contain sanitized
logs and interface counters, not access credentials or private system dumps.

The optional GUI binds to loopback; access it remotely through an authenticated
SSH tunnel. Do not expose its HTTP port to an untrusted network. Local channel
ownership is cooperative: it cannot prevent unrelated processes from sending CAN
frames. Software checks do not replace hardware emergency stopping.
