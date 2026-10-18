# osc001_api

USB オシロスコープ基板 osc001 を、Linux から Python・C・C++ で操作するためのライブラリです。
2ch のスコープ(最大 33 kSPS/ch、入力 ±24 V)、4ch のロジックアナライザ(最大 979.4 kHz)、
設定に依らず電圧を直接読むモニタ(op9)を扱えます。

> **非公式のソフトウェアです。** osc001 は秋月電子通商が販売していた USB オシロスコープ基板
> 「OSC001 PCB SCOPE」で、装置は USB 上で「DPScope SE」と名乗ります。本ソフトウェアは、
> 販売元・製造元・DPScope の設計者のいずれとも関係がなく、承認や支援も受けていません。
> 製品名・商標はそれぞれの所有者に帰属します。プロトコルは実機の挙動を観察・計測して
> 独自に調べたものです。


## デバイスの要点

| 項目               | 内容                                                                                                                                                                    |
| ------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| USB                | HID、VID:PID = **`04d8:f891`**、IN/OUT 64 バイトインタラプト、レポート ID なし                                                                                          |
| Linux              | `/dev/hidraw*` 経由(hid-generic バインド済み)。追加ドライバ不要、権限のみ必要                                                                                           |
| 取得モデル         | **op5(SET_REG 20 バイト書込み)= 設定適用兼 arm** → 記録は 192/fs 秒で完了し op6 が立つ(途中読みも可)→ op8 chunk0..5 = 384 バイト = **2ch × 192 サンプル**(インターリーブ、偶数番=CH1) |
| サンプル           | `値 = byte − 0x7F`                                                                                                                                                      |
| fs(統一式)         | `fs = 3e6 / (max(D, 61.56) + 29)`、到達域 **0.36 SPS 〜 33.13 kSPS/ch**                                                                                                 |
| レンジ             | ÷8 / ×1 タップ × ゲイン段。V/LSB = 47.9 mV(÷8)・5.04 mV(×1)× 2^段、両 ch 共通                                                                                     |
| 縦軸               | 8 bit 値 = 10 bit ADC 値 m の切り出し。ch ごとに違うのは 0 V の m だけ(可変抵抗 R12/R13)→ `calibrate_zero()`                                                           |
| op9 モニタ         | `monitor()`: 設定に依らず ±24 V を 47.9 mV 刻みで直接読む(約 1 ms/回)                                                                                                                          |
| ステータスフッター | 全応答の末尾 26 バイト(13 語 LE16)。電源サイクル検出(静的語)。w05/w07 は HID トランザクション数のパリティ                                                               |

## セットアップ(Linux / Raspberry Pi)

装置は `/dev/hidraw*` として見えます。一般ユーザーで開けるように udev ルールを置きます
(`/etc/udev/rules.d/70-osc001.rules`)。

デスクトップ(ログイン中のユーザーだけに許可する。おすすめ):

```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="f891", TAG+="uaccess"
```

画面のない Raspberry Pi など(`plugdev` グループの人に許可する):

```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="f891", GROUP="plugdev", MODE="0660"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`MODE="0666"` は、その PC の全ユーザー(リモートのユーザーを含む)に装置の操作を許すので勧めません。

## クイックスタート

```python
import sys
sys.path.insert(0, "python")   # 利用する側のリポジトリからは import osc001 で使える
import osc001

with osc001.Scope() as sc:     # /dev/hidraw を自動探索。明示も可: Scope("/dev/hidraw2")
    t = sc.capture()           # arm → 完了待ち → chunk0..5 読出の 1 発取得
    print(t.rate_hz, t.samples["ch1"][:8])
    volts = t.to_volts(1)      # 0 V 基準の電圧(取得時の zero_code を引く)
    print(t.rail_hit(1), t.clipped(1))   # レール到達・クリップの健全性検査
```

- 完全版取得(chunk6=フッター付き): `capture_deep()` → `t.footer / t.stale / t.power_cycle / t.filled_ff`
- レンジ・fs 設定: `set_range(ch, v_per_div)` / `set_sample_rate(hz)` / `warnings()`
- ハードトリガ: `set_trigger("ch1", "rising", level_v=2.5, mode="auto")` / `set_trigger("ext", "falling")` /
  `set_trigger(None)`(フリーラン)。源は CH1 と EXT(T 入力)だけで、CH2 のハードトリガは無い。
  トリガ点 = サンプル 0。Normal で未成立なら `TriggerTimeout`、Auto は未成立ならフリーランで取り直す
  (`t.triggered` = True / False / None)。レベルの電圧換算は `zero_code(ch)` / `volts_to_code(ch, v)`
- LA(ロジックアナライザ)モード: `la_set_rate(hz)` → `la_capture(channels=(2,), edge="falling")` → 844 サンプル
  (fs は 979.4k/490.9k/200k の固定段と ≦108.7 kHz の可変域、サンプル 0 = 発火点)。待機の中止は `la_abort()`(未発火のエッジ arm を残すとスコープ復帰後に USB 切断するため、
  `arm()` も自動で解除する)
- 0 V 合わせ: 入力を GND につないで `calibrate_zero()`(op9 と 6 レンジのスコープで 0 V の m を測り直す)。
  結果は `set_zero_m0(ch, tap, m0)` で戻せる
- op9 モニタ: `monitor(tap=8, n=10)` → `{"ch1": V, "ch2": V, ...}`(ロガー向け。スコープ取得中でも可)
- 遅い時間軸の逐次表示: `arm()` → `read_partial()` を繰り返す(`t.valid` までが書き込み済み。フリーランのみ)

## 安全と注意

- **入力は ±24 V まで。** 入力は抵抗分圧と保護ダイオードを通りますが、それを超える電圧はかけないでください。
- **osc001 の GND は USB を通して PC の GND とつながっています。** 測る回路の GND と PC の GND の間に
  電位差があると、GND 線に電流が流れて機器を壊すおそれがあります。商用電源に直接つながった回路や、
  GND が浮いていない別電源の回路を測るときは、絶縁(USB アイソレータなど)を検討してください。
- **装置がハングする操作があります。** LA の `edge="auto"` で監視 ch を指定する組み合わせはファームウェアが
  止まるので、API が拒否します。また、未発火の LA エッジ待ちを残したままスコープに戻ると USB が切れるので、
  `arm()` と `la_abort()` が自動で解除します。`set_raw()`・`osc001_raw_command()` などの脱出口は
  このガードの外にあるので、使う場合は自己責任でお願いします。ハングしたときは USB の抜き差しで戻ります。
- 本ソフトウェアは現状のまま提供され、いかなる保証もありません(LICENSE の第 7・8 条)。

## テスト

```bash
python3 -I python/test_osc001_math.py    # オフライン(デバイス不要)
python3 -I python/demo_capture.py        # 実機デモ(信号源あり前提)

cd c && make check                       # C/C++ オフラインテスト
cd c && make && ./build/demo_capture     # C 実機デモ(C++ 版は ./build/demo_capture_cpp)
cd c && ./build/lock_check               # 実機で flock 排他と open エラー区別を確認
```


## ディレクトリ構成

| ディレクトリ | 内容                                                                                                  |
| ------------ | ----------------------------------------------------------------------------------------------------- |
| `python/`    | API ライブラリ `osc001.py` + デモ + オフラインテスト                                                  |
| `c/`         | C API `libosc001`(include/src)+ C++ ラッパ `osc001.hpp` + hidraw トランスポート + モックテスト + デモ |

## ライセンス

[Apache License 2.0](LICENSE)
