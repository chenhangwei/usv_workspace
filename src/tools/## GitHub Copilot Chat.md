## GitHub Copilot Chat

- Extension: 0.57.0 (prod)
- VS Code: 1.129.1 (8a7abeba6e03ea3af87bfbce9a1b7e48fed567b8)
- OS: linux 6.6.123.2-microsoft-standard-WSL2+ x64
- Remote Name: wsl
- Extension Kind: Workspace
- GitHub Account: chenhangwei

## Network

User Settings:
```json
  "http.systemCertificatesNode": true,
  "telemetry.telemetryLevel": "all",
  "github.copilot.advanced.debug.useElectronFetcher": true,
  "github.copilot.advanced.debug.useNodeFetcher": false,
  "github.copilot.advanced.debug.useNodeFetchFetcher": true
```

Connecting to https://api.github.com:
- DNS ipv4 Lookup: timed out after 10 seconds
- DNS ipv6 Lookup: timed out after 10 seconds
- Proxy URL: None (0 ms)
- Electron fetch: Unavailable
- Node.js https: timed out after 10 seconds
- Node.js fetch (configured): timed out after 10 seconds

Connecting to https://api.githubcopilot.com/_ping:
- DNS ipv4 Lookup: 140.82.113.22 (9073 ms)
- DNS ipv6 Lookup: Error (167 ms): getaddrinfo ENOTFOUND api.githubcopilot.com
- Proxy URL: None (1 ms)
- Electron fetch: Unavailable
- Node.js https: HTTP 200 (844 ms)
- Node.js fetch (configured): HTTP 200 (5484 ms)

Connecting to https://copilot-proxy.githubusercontent.com/_ping:
- DNS ipv4 Lookup: 138.91.182.224 (1139 ms)
- DNS ipv6 Lookup: Error (60 ms): getaddrinfo ENOTFOUND copilot-proxy.githubusercontent.com
- Proxy URL: None (14 ms)
- Electron fetch: Unavailable
- Node.js https: HTTP 200 (1014 ms)
- Node.js fetch (configured): HTTP 200 (6531 ms)

Connecting to https://mobile.events.data.microsoft.com/OneCollector/1.0?cors=true&content-type=application/x-json-stream (Node.js fetch): HTTP 200 (1749 ms)
Connecting to https://telemetry.individual.githubcopilot.com/telemetry (Node.js https): HTTP 200 (4497 ms)
Connecting to https://default.exp-tas.com/vscode/ab (Node.js fetch): HTTP 200 (660 ms)

Number of system certificates: 363

## Notes

- Active fetcher: Node.js fetch.
- For corporate networks also see: [Troubleshooting firewall settings for GitHub Copilot](https://docs.github.com/en/copilot/troubleshooting-github-copilot/troubleshooting-firewall-settings-for-github-copilot).