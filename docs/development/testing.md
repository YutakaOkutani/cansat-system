# Deprecated Testing Link

> **Audience: AI agents following a legacy link.** Do not load this page in addition to its owners.

## 実行コマンド一覧

- Human setup and execution: [`../../README.md`](../../README.md)
- AI diagnosis, test tiers, and the single full spec command: [`../06-debug-and-test-rules.md`](../06-debug-and-test-rules.md)

## 超音波センサ（HC-SR04）の安全な接続

Human wiring and diagnostic instructions: [`../../README.md#531-超音波センサの使い方`](../../README.md#531-超音波センサの使い方).

Before powering or testing the sensor, load [`../reference/hardware-safety.md`](../reference/hardware-safety.md). It owns the ECHO input-voltage constraint: direct connection for a verified GPIO-compatible output, or level conversion for a 5 V output.

This compatibility page preserves existing anchors in the human-facing root `README.md`.

## AI Checklist

- Am I using `06-debug-and-test-rules.md` for testing decisions?
- Did I load `reference/hardware-safety.md` before physical sonar work?
