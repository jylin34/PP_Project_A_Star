# Parallel A* Research Framework v1.0

**C++17 + POSIX Threads + Python standard library** | 四種最短路徑搜尋 | Grid + Hsinchu OSM | 一鍵驗證、測試、分析與 SVG 圖表。

這是針對 NYCU 平行程式設計期末專題打造的 **from-scratch reference-inspired experiment framework**。演算法原理參考 Jinnai & Fukunaga (AAAI 2016) 的 Hash-Distributed A* / Abstract Zobrist 工作分配研究，但 **沒有複製該專案的程式碼，亦不聲稱重現原論文所有資料集與優化**。

- Reference repository: https://github.com/jinnaiyuu/Hash-Distributed-Astar
- Paper: https://jinnaiyuu.github.io/pdf/papers/AAAI-16%20Jinnai-Fukunaga.pdf

## 1. 快速開始（Ubuntu / WSL2）

安裝環境：

```bash
sudo apt update && sudo apt install -y cmake build-essential python3
```

**直接跑全部預設 Grid 實驗**（不需要任何 pip 套件）：

```bash
chmod +x run_all.sh run_smoke.sh
./run_all.sh
```

先用 Grid + 內建的有向道路測資做整合驗證：

```bash
./run_smoke.sh
```

正式 Grid（多種 Query，非只有固定左上至右下）：

```bash
./run_all.sh --dataset grid \
  --sizes 128 256 512 --seeds 7 42 100 \
  --max-grid-queries 6 --density 0.2 \
  --threads 1 2 4 8 --hash-policies zobrist azobrist abstract mixed \
  --tile 8 --batch 32 --repeats 5 --out results/grid_full
```

**注意：這是一個相對大的矩陣。先跑 smoke，再跑正式實驗。**

## 2. 使用你之前下載的新竹 OSM 資料

把你已有的檔案放到：

```text
data/osm/nodes.csv      # id,lat,lon
data/osm/edges.csv      # from,to,weight (meters; directed)
data/osm/queries.csv    # start,goal
```

不需要重新下載 GraphML，也不必再執行舊的 `.graph` 中繼格式轉換器。

```bash
./run_all.sh --dataset osm \
  --threads 1 2 4 8 --hash-policies zobrist azobrist abstract mixed \
  --max-queries 20 --repeats 5 --out results/hsinchu_v1
```

你原本的 4,887 Nodes、12,160 Directed Edges、20 Queries 可以直接適用，只要三份 CSV 保持先前格式即可（資料未包含在本 ZIP 中）。

也可以使用已有 GraphML 重新產生 CSV：

```bash
uv venv && uv pip install osmnx
uv run --no-sync python tools/fetch_osm.py \
  --graphml data/osm/hsinchu.graphml --queries 20
```

第一次下載新竹路網（需連線 OSMnx／Overpass）：

```bash
uv run --no-sync python tools/fetch_osm.py \
  --center-lat 24.804 --center-lon 120.971 --radius-m 8000
```

下載和 Benchmark 是**分開的**：正式量測不會存取網路，避免資料版本改變。

## 3. 演算法

| Variant | CLI | Threading | Correctness |
|---|---|---|---|
| Sequential A* | `sequential` | 1 | Dijkstra validated |
| Bidirectional A* | `bidirectional` | 1, dual search direction | Two heuristics, safe lower-bound stop |
| Shared-Queue A* | `shared` | Pthreads 1–128 | mutex-protected global heap, striped state records |
| HDA* | `hda` | Pthreads 1–128 | owner-only records/open; per-owner inbox; atomic work credits |

**Bidirectional HDA*** 和 Two-thread Bidirectional **尚未實作**，保留為 Potential Extension；目前 `bidirectional` 是單執行緒雙向搜尋。

HDA* 支援五種分配策略：

- `mod`: vertex-ID modulus (naive baseline)
- `mixed`: 64-bit mixed vertex-ID hash (non-spatial baseline)
- `zobrist`: XOR over two deterministic fine spatial feature keys
- `abstract`: hash of spatial cell (locality-oriented abstraction)
- `azobrist`: XOR over deterministic coarsened spatial feature keys

**重要研究界線**：Grid 的 fine features 是 x/y 格點；OSM 的 fine features 是約 100m 尺度的位置網格。`azobrist` 是**借鑑原論文思想、針對本專題空間圖狀態設計的變體**，不是原作者多個 benchmark domain 專用的完整 AZH 特徵設計，也不是逐行移植原碼。`--tile` 設定抽象倍率（預設 8），`--batch` 設定每目的地 Buffer 上限，`--flush-interval` 設定最多累積幾次 Node Expansions 才將 Sender Buffers 送出（預設 8）。

### 長時間實驗中斷後接續

每完成一次 Search 就即時寫入 `runs.csv`，不必等待上千筆全部結束。使用同一組參數加上 `--resume`：

```bash
./run_all.sh --dataset osm --threads 1 2 4 8 \
  --hash-policies zobrist azobrist --repeats 5 \
  --out results/hsinchu_v1 --resume
```

重啟時會核對 Experiment Config、C++ Binary SHA256、OSM Input SHA256，避免不同版本或資料混在同一份 CSV。**非 `--resume` 執行會覆寫同路徑 runs.csv**，請替不同實驗指定不同的 `--out`。

## 4. Output（`--out results/...`）

```text
runs.csv                 # 每次搜尋；每次由 C++ 驗證 Dijkstra + returned path
per_query_summary.csv    # 先在相同 query 的 repeats 中取 median，再計算相對比率
summary.csv              # 跨 query 的 median（避免用 aggregate ratios 造成偏差）
manifest.json            # timestamp / arguments / platform / compiler / CSV SHA256
experiment_config.json   # resume safety signature
REPORT.md                # 自動生成的報告用統計表和解讀注意事項
*_runtime.svg
*_speedup_vs_sequential.svg
*_scaling_speedup.svg
*_parallel_efficiency.svg
*_expanded.svg
*_communication_overhead.svg
*_search_overhead.svg
*_load_balance.svg
*_expansion_imbalance.svg
```

SVG 使用 Python Standard Library 生成，沒有 pandas、matplotlib 依賴。可直接在瀏覽器查看，也能插入報告。

**主要公式**（逐 Query 基準）:

- Communication Overhead, `CO = remote_messages / generated_nodes`
- Search Overhead, `SO = expanded_parallel / expanded_sequential - 1`
- Load Balance, `LB = max(owner assigned candidate nodes) / avg(owner assigned candidate nodes)`（HDA*）；另提供 Expansion Imbalance
- Speedup vs sequential, `S_seq(P) = T_seq(1) / T_algorithm(P)`
- Scaling speedup, `S_scale(P) = T_algorithm(1) / T_algorithm(P)`
- Scaling efficiency, `E_scale(P) = S_scale(P) / P`

更多方法學限制和欄位定義參見 `docs/EXPERIMENT_PROTOCOL.md`。

## 5. 單次搜尋 / 除錯

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure

./build/parallel_astar --algorithm hda --grid 64 --seed 7 --density 0.2 \
  --start 0 --goal 4095 --threads 4 --hash azobrist --tile 8 --batch 16 --verify 1

./build/parallel_astar --algorithm sequential \
  --nodes data/osm/nodes.csv --edges data/osm/edges.csv \
  --start 56 --goal 1324 --verify 1
```

每次搜尋輸出一行 JSON，benchmark runner 會整理為 CSV。`--verify 1` 比對 Dijkstra 最短路徑與實際有向 Path 的總成本；Dijkstra 驗證在搜尋計時區間**之外**進行。

## 6. 進一步研究與效能界線

- 第一版採 **incumbent-based pruning + global quiescence**。找到 goal 不會直接停止；所有 queued/active/inbox work 必須正確完成。這比論文中的低成本分散式終止偵測保守，不能直接宣稱等同作者的 termination protocol。
- HDA* 依參考實作使用 async inbox，並加入 **persistent sender-side message batching**，可跨數次 Expansion 累積。每筆 Buffered Work 在建立時先登記 Outstanding Credit，達到 Batch Size、Flush Interval 或 Worker 即將 Idle 時才一次送出。這比每條邊送一個 Message 更接近參考實作的 Asynchronous Buffering。
- Thread creation / join **包含在 time_ms**；Graph CSV parsing、圖生成、Dijkstra verification **不計入**。對極短搜尋而言，Thread Startup 很可能主導結果。
- WSL2 的 CPU Scheduling 會影響效能；最終需要在相同裸機／學校 HPC Linux 多核心環境重新實驗。
- 不要只看 Speedup。仔細比較 CO、SO、LB、Expanded Nodes、Hash policies、Batch Size。

## 7. 目錄

```text
include/astar.hpp          shared graph/search/results APIs
src/graph.cpp              synthetic grid, directed road CSV, admissible heuristic
src/search.cpp             sequential, Dijkstra, one-thread bidirectional
src/parallel.cpp           pthreads shared-queue and owner-managed HDA*
src/main.cpp               JSON CLI
tests/test_search.cpp      deterministic graph, thread, hash correctness matrix
tools/benchmark.py         reproducible experiment orchestration and summary
tools/plot_results.py      dependency-free SVG chart generator
tools/fetch_osm.py         optional OSMnx importer/downloader
data/osm/sample_*.csv     sample directed graph, NOT Hsinchu data
docs/                     architecture, experiment protocol, reference notes
```

### Acknowledgements

Algorithmic inspiration: Jinnai & Fukunaga, "Abstract Zobrist Hashing: An Efficient Work Distribution Method for Parallel Best-First Search," AAAI 2016. Original repository is MIT-licensed. **This project is a new implementation; no source files copied from the reference repository.** OSM-based datasets require **© OpenStreetMap contributors** attribution and ODbL compliance.
