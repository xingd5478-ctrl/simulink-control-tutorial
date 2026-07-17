# Security Policy

## Supported Versions

| Version | Supported          |
|---------|--------------------|
| 1.1.x   | :white_check_mark: |
| 1.0.x   | :white_check_mark: |
| < 1.0   | :x:                |

## Reporting a Vulnerability

If you discover a security vulnerability in this project, please report it
by opening an issue on GitHub. Do NOT disclose it publicly until the maintainer
has had a chance to address it.

## Scope

This project is an educational tutorial repository. Security concerns are
limited to:

- Malicious `.m` or `.slx` files that could execute harmful commands
- Code generation output that introduces runtime vulnerabilities

## Best Practices

- Always review `.m` scripts before running them
- Do not run `.slx` models from untrusted sources
- Treat generated C code with the same scrutiny as hand-written code
