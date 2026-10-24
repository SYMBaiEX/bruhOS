# Local trace dashboard

The dashboard inspects saved bruhOS episodes without starting a brain, opening
hardware, or sending robot commands. It uses Python's standard library and the
existing trace loader and verifier; no API key or voice service is required.

From an installed bruhOS checkout:

```sh
bruhos run "Wave hello." --traces traces
bruhos dashboard --traces traces
```

Open http://127.0.0.1:8765/. Use `--port` to select another port. The trace path
must match the runtime's trace output directory. An empty directory is supported.
For an existing Pi installation, pass its trace directory, for example
`--traces ~/.local/share/bruhos/traces`.

The console refreshes every two seconds and shows the latest 12 readable episodes,
sorted by start time. It scans episode headers but loads/verifies step histories
only for the displayed page. Status responses contain the displayed summary fields,
without plans or raw observations. Filter completed or stopped/failed episodes and expand each
record for its brain, result, recovery count and full trace root. Malformed or
partially written episodes are skipped and retried on the next refresh. Episode
instructions are displayed, so use only traces appropriate for the viewer.

Verification calls the existing `verify_episode` implementation. Its hash chain
does not cover all header metadata and does not independently establish physical
completion. Outcomes are reported by the runtime/adapter. This dashboard does not
report live runtime, voice, camera or IMU status.

The server binds to loopback only and serves bundled assets and `/status`, never
arbitrary trace files. For remote viewing, use an SSH tunnel:

```sh
ssh -N -L 8876:127.0.0.1:8765 pi@ROBOT_IP
```

Then open http://127.0.0.1:8876/. Stop the server with Ctrl+C. No system services or
desktop launchers are installed. Assets work offline; artwork provenance and font
licenses are in `web/assets/`.
