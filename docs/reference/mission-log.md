# 最終接近・終了・復帰診断CSV（schema 6）

`mission/const.py` の `LOG_HEADER` と `mission/diagnostics.py` が列定義を持つ。
既存列の順序は維持し、診断列を末尾に追加した。旧CSVには列を補完して
成功・未割り込みと推測せず、解析レポートで `unknown` と表示する。

## 終了の読み方

- `MissionEndReason`: ミッションの結果。後からCtrl+Cしても上書きしない。
- `Phase=7`: 終端状態への遷移。ハンドラが実行された保証はない。
- `Phase7HandlerEntered`: Phase7ハンドラへ実際に入った場合のみ1。
- `TerminalLEDCommand`: Phase7のLED指令。実際の発光や消灯後の保持を保証しない。
- `ShutdownReason`: 最初の終了要求の理由。`KEYBOARD_INTERRUPT`、
  `RUN_EXIT`、ミッション結果など。`RUN_EXIT`は正常成功の意味ではない。
- `InterruptCount/Phase/ElapsedSec/Stage`: 捕捉したCtrl+Cの回数と最後の受信位置。
  Phase7遷移後や後片付け中のCtrl+Cもミッション結果とは別に残す。
- `ShutdownStage`: `not_requested` → `radio_restore` → `hardware_close`
  → `manifest_finalize` → `completed`。各処理に入る前にCSVをflushする。
  Phase7からは先に`phase7_handler`を設定し、センサの停止を通知する。
- `ShutdownCompleted`: 終了ルーチンが最後まで到達すると1。`ShutdownError`も確認する。
- `HardwareCloseDevice`: これからcloseするデバイス。途中で止まった場合の位置。
- `HardwareCloseCompleted`: デバイスの解放処理ループを完走した場合に1。
  個別closeの失敗・カメラcloseの時間切れは`ShutdownError`に残る。
- `ShutdownError`: 最後に捕捉した後片付けのエラー分類。空欄は記録なし。
- `RunExceptionType`: main run境界を通った通常例外の型名。
- `RunFinallyReached`: runのfinallyに到達した場合に1。

`ShutdownCompleted`や`RunFinallyReached`はOSの終了コードではない。
プロセス終了後の証明には外部ランナーやサービス側の終了コードが必要。
強制電源断・SIGKILLはPythonで捕捉できず、完了フラグなしの最終行となり得る。

## Phase7の自動終了

成功・失敗・未知の理由を問わず、Phase7は終端。通常の無線監視や時間切れ判定より
先に処理し、Phase6やタイムアウト処理がPhase7へ変更した場合も同じ制御周期で
終了処理に入る。LED処理の例外でもfinallyから後片付けを実行する。
センサワーカーの停止通知と、終了処理の開始済みフラグは分離している。

`main.py`と`runs/orch/`、単独フェーズ実行が使う`mission/run.py`は、
Phase7処理開始またはその他の終了要求時から15秒のプロセス終了監視を有効にする。
通常はそのままreturnして終了する。デバイスclose、ログI/O、ドライバの非daemon
スレッド等で終了が滞った場合は、最大0.2秒の診断出力猶予後に強制終了する。
Phase7到達済みなら終了コード80、それ以前ならコード1とする。
この経路は後片付けの完了を保証せず、正常終了として扱わない。
可能なら`ShutdownStage=forced_exit`と
`ShutdownError=process_exit_deadline:<直前の段階>`をCSVに残す。
ログI/O自体が停止している場合はこの最終記録も保証できない。
正常に終了したプロセスでは監視daemonも消滅するため、強制終了は起こらない。

ライブラリとして直接生成したControllerにプロセス全体を終了させる監視は付かない。
通常の実行入口を使用する。配布例は`Restart=on-failure`と`RestartSec=5`を維持し、
`RestartPreventExitStatus=80 81`を追加している。Phase7到達後のPython例外・
捕捉済みの後片付けエラー・終了期限超過はコード80で終了し、systemdは再起動しない。
Phase7での通常終了はコード0。GOAL成否は従来どおり`MissionEndReason`で判断する。
Phase7以前の例外や終了期限超過は通常の異常終了として自動再起動の対象となる。
OS全体の電源断・再起動には下記の永続チェックポイントと復帰用serviceを使用する。
保存状態のない初回ミッションは従来どおりtimerの5分待ちで開始する。

以前の案の`Restart=no`を実機に設定済みなら、`sudo systemctl edit cansat.service`で
以下に修正し、`sudo systemctl daemon-reload`で反映する。コード80を扱う実装とセットで更新する。

```ini
[Service]
Restart=on-failure
RestartSec=5
RestartPreventExitStatus=80 81
```

`systemctl show cansat.service -p Restart -p RestartPreventExitStatus`で有効設定を確認する。
この操作だけではミッションを起動しない。

## Phase6・停止・パルス

`Phase6Stage/Action/Gate`、経過・観測待ち・静定残り時間、要求パルス数・票数を記録する。
Actionは最後に要求したパルス方向で、Stageがmoveでなければ駆動要求ではない。
Gateは静定待ち、停止後のサンプル待ち、新しい組待ち、証拠却下、票採用、
補正確認待ち、パルス要求、方位変化、終端などを区別する。

`GoalEval*`はPhase6判定時の画像Seq・距離Seq・距離・観測時刻差・理由・確認数・
判定時刻をまとめて保持する。通常のセンサ列は別時点の最新値なので混同しない。
`GoalEvalMatched=1`は接近または補正用の証拠が成立したことを示し、GOAL確定ではない。
例えば軸外対象の補正では、Reasonが`cone_off_axis`でもMatchedは1になり得る。

`CloseTrackAnchorPresent/CenterPresent/RemainingSec/CenterHeadingDeg`は
最新センサスナップショットの追跡履歴・中央観測履歴・有効期限残り・基準方位。
既存の`ConeCloseTrackEligible/Hold/Reason`と併用する。

`MotorStopReason`は停止要求の分類。Phase5・6・7と終了要求を重点的に記録し、
その他の呼び出しは`unspecified`とする。`Phase6MotorGate`はモータ側の抑止理由。
パルス終了後は通常の停止要求で上書きされるため、最後のパルス停止理由は
`Phase6LastPulseStopReason`に別途保持する。

`Phase6PulseStartedCount`は実出力指令の開始累計（実測回転数ではない）。
`Phase6LastPulseId/StartElapsedSec/EndElapsedSec/DurationSec`は最後の出力区間を保持し、
50 msパルスが定期CSVの間に終わっても情報が消えないようにする。
要求数と開始数、実出力時間、抑止理由を照合する。開始数はrun内累計、要求数・IDは
Phase6への入場ごとにリセットされる。CSV間で複数パルスが走った場合は最後の区間のみ残る。

経過時間列の単位は秒、距離はcm、方位は度。未観測は空欄、真偽値は0/1。
Phase6は1秒に1回および終端判定時に状況を即時表示する。
Phase7・終了段階の表示もflushし、バッファ内に残りにくくする。

## 解析

`analysis/log.py`、`analysis/explorer.py`の両方が既存出力に加えて
`final_approach_timeline.csv`と`final_approach_summary.txt`を作成する。
前者は判定・モータ・終了診断の時系列、後者はPhase6/7初記録と終了理由の要約。
schema 3以前でも生成できるが、割り込みや後片付けの未記録情報は復元できない。

## Phase 0の高度診断（schema 5で追加）

- `Phase0CurrentAltitude`: Phase 0の判定に使用した有効なBMP180現在高度 [m]。
- `Phase0MaxAltitude`: Phase 0開始後の有効な観測高度の最大値 [m]。
- `Phase0AltitudeDrop`: 最高高度 − 現在高度 [m]。20 m以上で高度条件成立。

未取得時は空欄。BMPが無効・古い場合、現在高度と下降量は空欄とし、最高高度は保持する。
Phase 0再開始時にリセットし、終了後は最後の判定値を保持する。
高度条件の20秒継続、衝撃検知による遷移、タイムアウトによる遷移は従来どおり。
`analysis/log.py`は3列をPhase 0高度グラフに表示する。旧CSVでは存在しない列を省略する。

## 中断からの復帰（schema 6）

対象は本番入口`main.py`のみ。`mission/recovery.py`が
`~/cansat-data/mission-state.json`を管理する。新規ミッションはハードウェア初期化前に
P0を保存し、以後は約1秒間隔、フェーズ・P2段階・P6前進要求回数の変化、終了時に保存する。
一時ファイルをfsyncしてrenameし、親ディレクトリもfsyncする。
ロックファイルによって同じ状態を使う二重実行・実行中のresetを拒否する。

- P0〜P6: 最後に保存した同じフェーズに再入場する。命令位置やPWMは復元しない。
- P7: 成否・理由を問わず、ハードウェア構築前に終了する。通常再起動・シャットダウン時のリセットを除き、記録は自動削除しない。
  P7遷移時はモータ停止後、LED・後片付けより先に保存する。
- 引き継ぐ情報: ミッションID、直前RunId、復帰回数、消費済み全体時間・各フェーズ時間、
  同フェーズ滞在時間、P2段階と消費時間、P6前進要求回数。
- 全体時間には電源断・OS起動・初期化時間も加算する。フェーズの実行時間には停止中を加算しない。
  復帰後も既存の時間切れ・前進回数制限を使う。
- センサ値・画像追跡・GOAL票・方位補正・P0高度ピークなど観測履歴は持ち越さない。
  P2で完了済みの脱出段階は繰り返さず、未完了の学習は新しい観測で行う。
  P3では既存のGPS方位フォールバックを使う。
- 復帰準備と最初の制御周期の保存が終わるまでモータ出力を許可しない。
  目標座標・無線設定の変更、破損データ、時計の2秒超の巻き戻りは復帰を拒否する。
  コード81はsystemdの再起動対象外。状態ファイルは残す。
- 部分実行`runs/orch/`はチェックポイントを利用しない。

CSVの`RecoveryMissionId`は一連の復帰を束ねるID、`RecoveryCount`は復帰回数（初回0）、
`RecoveryFromRunId`/`RecoveryFromPhase`は直前の実行・保存フェーズ。
復帰ごとに新しいRunId・run bundleを作り、manifestにも同じ情報を記録する。
解析の最終接近レポートにもこれらの列を表示する。

`cansat-resume.service`は状態ファイルがあるOS起動で同じ`cansat.service`を開始する。
これによりtimerの5分待ちを省略する。P7保存済みでも入口の確認は動くが、走行はしない。
従来のtimerも同じserviceを指すので並行起動はしない。
P7終了後にtimerが発火しても、アプリのP7確認で即終了する。
実行中のプロセス異常には従来どおり`Restart=on-failure`と5秒待ちを使う。

新しい試走は、リセット用unitの有効化後なら通常の`sudo reboot`だけで開始できる。
再起動しない場合はサービスとtimerを停止したうえで、実行ユーザーのPythonで
`python -m mission.recovery reset`を実行する。`status`は停止中の記録確認用。
配布unitの配置・有効化手順はREADMEを参照。

限界: 保存直前の電源断では前回保存位置から再開するため、短い動作を繰り返す可能性がある。
物理動作を厳密に一度だけ実行する保証はない。P7も永続保存を完了できなかった場合には
記録が残らない。ストレージ書込みやOS時計の正常性、実機での電源断復帰は別途確認する。

## 通常の再起動・シャットダウンを新規試走として扱う例外

`cansat-reset-on-reboot.service`を`reboot.target`と`poweroff.target`にenableする。
`DefaultDependencies=no`とし、ミッションservice・timer・復帰用serviceとの
`Conflicts`と`After`によって書込み元の停止を待つ。その後、既存のロック付き
`mission.recovery reset`を実行し、P7を含む保存状態を削除してfsyncする。
`shutdown.target`・`umount.target`より前に処理する。ミッション側の判断処理や
`ExecStop`にリセットは追加しない。service単体の停止・異常再起動で記録を失わないためである。
依存順序の仕様は[systemd.unitの公式資料](https://github.com/systemd/systemd/blob/main/man/systemd.unit.xml)を参照。

次回は復帰用serviceのファイル存在条件が不成立となり、初回用timerによる5分待ち後にP0から開始する。
この判断はコマンドの入力者を識別しないため、`systemctl reboot`や自動化による通常再起動もリセットする。
`shutdown -h now`・`poweroff`による正常なシャットダウンでも保存状態を消し、次の電源投入ではP0から開始する。
瞬断・強制再起動はこの処理を通らず保存状態を維持する。処理途中の電断、ファイルシステム障害、
手動起動したミッションがロックを保持している場合など、resetを完了できなければ保存状態が残り得る。
新unitは`enable`のみで有効化し、`--now`でその場のリセットを行わない。

既存unitを更新する際は`daemon-reload`だけでは`poweroff.target`への登録は追加されない。
再配置とパス調整後、`systemctl enable cansat-reset-on-reboot.service`を再実行する。
手動で新規試走する場合は`systemctl start cansat-reset-on-reboot.service`の成功後に
`systemctl start cansat.service`を実行する。`&&`で接続し、reset失敗時は起動しない。
リセット用unitはOS再起動・電源断を要求する依存関係を持たず、手動startはリセットだけを行う。
