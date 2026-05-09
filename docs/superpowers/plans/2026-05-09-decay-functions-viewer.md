# 衰减函数可视化对比页面 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `docs/index.html` 创建一个零构建、纯前端的静态页面，叠加显示 PID 库 5 个衰减函数曲线，并允许用户实时调节参数。

**Architecture:** 单文件 HTML，CDN 引入 Chart.js v4 与 KaTeX。函数定义、状态、渲染都在内嵌 `<script>` 中完成。每次参数/勾选变化通过 `requestAnimationFrame` 节流后重算 200 个采样点并更新 Chart 数据集。无构建工具、无依赖管理。

**Tech Stack:** HTML5 + CSS3 + 原生 ES2017 JavaScript；Chart.js 4.x（CDN）；KaTeX 0.16.x + auto-render（CDN）。

---

## 文件结构

- **创建：** `docs/index.html` — 单文件全部内嵌
- **修改：** `README.md` — 顶部加在线对比页面链接

`docs/index.html` 内部分块（同一文件内的逻辑章节，便于阅读）：

| 章节 | 行/位置 | 责任 |
|---|---|---|
| `<head>` | 顶部 | 标题、CDN link/script、内嵌 CSS |
| `.app-header` | body 顶 | 标题 + GitHub 链接 |
| `.function-picker` | header 下 | 5 个复选框 + 重置按钮 |
| `.chart-container` | 中部 | `<canvas>` + Chart.js |
| `.params-panel` | 底部 | 动态生成的参数块 |
| `<script id="data">` | body 末 | 函数定义 + 默认参数（FUNCS 表） |
| `<script id="ui">` | body 末 | 状态、渲染、事件绑定 |

---

## Task 1: 创建 HTML 骨架与 CDN 引入

**Files:**
- Create: `docs/index.html`

- [ ] **Step 1: 创建 HTML 骨架文件**

写入以下完整内容到 `docs/index.html`：

```html
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>PID 衰减函数对比</title>

  <link
    rel="stylesheet"
    href="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css"
    integrity="sha384-nB0miv6/jRmo5UMMR1wu3Gz6NLsoTkbqJghGIsx//Rlm+ZU03BU6SQNC66uf4l5+"
    crossorigin="anonymous"
  />
  <script
    defer
    src="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js"
    integrity="sha384-7zkQWkzuo3B5mTepMUcHkMB5jZaolc2xDwL6VFqjFALcbeS9Ggm/0T2VMjhKTFUR"
    crossorigin="anonymous"
  ></script>
  <script
    defer
    src="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/contrib/auto-render.min.js"
    integrity="sha384-43gviWU0YVjaDtb/GhzOouOXtZMP/7XUzwPTstBeZFe/+rF4y8Q+kg7nmUWN6N5g"
    crossorigin="anonymous"
  ></script>

  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.6/dist/chart.umd.min.js"></script>

  <style>
    /* 占位，Task 2 填充 */
  </style>
</head>
<body>
  <header class="app-header">
    <h1>PID 衰减函数对比</h1>
    <a href="https://github.com/MisakaMikoto128/PID" target="_blank" rel="noopener">GitHub</a>
  </header>

  <section class="function-picker" id="functionPicker"></section>

  <section class="chart-container">
    <canvas id="chart"></canvas>
  </section>

  <section class="params-panel" id="paramsPanel"></section>

  <script id="data">
    /* 占位，Task 3 填充 */
  </script>
  <script id="ui">
    /* 占位，Task 4-7 填充 */
  </script>
</body>
</html>
```

- [ ] **Step 2: 在浏览器中验证骨架加载**

运行：双击 `docs/index.html` 或 `python -m http.server 8000` 后访问 `http://localhost:8000/docs/`。

预期：浏览器显示标题"PID 衰减函数对比"和"GitHub"链接，控制台无报错（CDN 资源 200）。

- [ ] **Step 3: 提交**

```bash
git add docs/index.html
git commit -m "新增衰减函数对比页面 HTML 骨架与 CDN 依赖"
```

---

## Task 2: 内嵌 CSS — 浅色主题与响应式布局

**Files:**
- Modify: `docs/index.html`（替换 `<style>` 内容）

- [ ] **Step 1: 替换 `<style>` 标签内容**

将 `<style>...</style>` 整段替换为：

```html
<style>
  :root {
    --bg: #ffffff;
    --fg: #1f2328;
    --muted: #6e7781;
    --border: #d0d7de;
    --accent: #0969da;
    --panel-bg: #f6f8fa;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", "PingFang SC",
                 "Hiragino Sans GB", "Microsoft YaHei", sans-serif;
    background: var(--bg);
    color: var(--fg);
    line-height: 1.5;
  }
  .app-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 16px 24px;
    border-bottom: 1px solid var(--border);
  }
  .app-header h1 { margin: 0; font-size: 20px; }
  .app-header a {
    color: var(--accent);
    text-decoration: none;
    font-size: 14px;
  }
  .app-header a:hover { text-decoration: underline; }

  .function-picker {
    display: flex;
    flex-wrap: wrap;
    gap: 12px 20px;
    align-items: center;
    padding: 16px 24px;
    background: var(--panel-bg);
    border-bottom: 1px solid var(--border);
  }
  .function-picker label {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    cursor: pointer;
    font-size: 14px;
    user-select: none;
  }
  .function-picker .swatch {
    display: inline-block;
    width: 12px;
    height: 12px;
    border-radius: 2px;
  }
  .function-picker button {
    margin-left: auto;
    padding: 4px 12px;
    font-size: 13px;
    background: white;
    border: 1px solid var(--border);
    border-radius: 4px;
    cursor: pointer;
  }
  .function-picker button:hover { background: #f3f4f6; }

  .chart-container {
    padding: 24px;
    height: 480px;
  }

  .params-panel {
    padding: 0 24px 32px;
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
    gap: 16px;
  }
  .param-block {
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 12px 16px;
    background: var(--panel-bg);
  }
  .param-block .title {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 600;
    margin-bottom: 6px;
  }
  .param-block .formula { margin: 8px 0 12px; font-size: 14px; }
  .param-row {
    display: grid;
    grid-template-columns: 24px 1fr 56px;
    align-items: center;
    gap: 8px;
    margin: 6px 0;
    font-size: 13px;
  }
  .param-row input[type="range"] { width: 100%; }
  .param-row .value { text-align: right; font-variant-numeric: tabular-nums; }

  @media (max-width: 900px) {
    .chart-container { height: 320px; padding: 16px; }
    .params-panel { padding: 0 16px 24px; grid-template-columns: 1fr; }
    .function-picker { padding: 12px 16px; }
    .app-header { padding: 12px 16px; }
  }
</style>
```

- [ ] **Step 2: 在浏览器刷新验证视觉**

预期：页面有清晰的 header（白底）、灰色复选框区域（暂无内容）、空白图表区域、空白参数区域。窗口缩到 600px 宽时布局保持单列、无横向滚动条。

- [ ] **Step 3: 提交**

```bash
git add docs/index.html
git commit -m "新增对比页面浅色主题样式与响应式布局"
```

---

## Task 3: 函数表与采样工具

**Files:**
- Modify: `docs/index.html`（填充 `<script id="data">`）

- [ ] **Step 1: 替换 `<script id="data">` 内容**

将 `<script id="data">...</script>` 整段替换为：

```html
<script id="data">
  // 5 个衰减函数的定义。注意 sigmoidabsx 按 PID.c 注释里的正确公式实现，
  // 而非当前 C 代码（C 代码缺少 "1+"，是已知 bug，不在本页面修复范围）。
  const FUNCS = [
    {
      key: "dsigmoidn",
      label: "dsigmoidn",
      color: "#e41a1c",
      formula: String.raw`y = 1 - \frac{4e^{-az}}{(1+e^{-az})^2} = \tanh^2(az/2)`,
      params: [{ name: "a", min: 0.1, max: 5, step: 0.01, default: 1.0 }],
      eval: (z, { a }) => {
        const e = Math.exp(-a * z);
        const q = 1 + e;
        return 1 - (4 * e) / (q * q);
      },
    },
    {
      key: "sigmoidabsx",
      label: "sigmoidabsx",
      color: "#377eb8",
      formula: String.raw`y = \frac{1}{1+b\,e^{-a|z|}}`,
      params: [
        { name: "a", min: 0.1, max: 5, step: 0.01, default: 1.0 },
        { name: "b", min: 0.1, max: 5, step: 0.01, default: 1.0 },
      ],
      eval: (z, { a, b }) => 1 / (1 + b * Math.exp(-a * Math.abs(z))),
    },
    {
      key: "tanhabsx",
      label: "tanhabsx",
      color: "#4daf4a",
      formula: String.raw`y = \tanh(a|z|)`,
      params: [{ name: "a", min: 0.1, max: 5, step: 0.01, default: 1.0 }],
      eval: (z, { a }) => Math.tanh(a * Math.abs(z)),
    },
    {
      key: "px1",
      label: "px1",
      color: "#984ea3",
      formula: String.raw`y = 1 - \frac{1}{\sqrt{a|z|+1}}`,
      params: [{ name: "a", min: 0.1, max: 5, step: 0.01, default: 1.0 }],
      eval: (z, { a }) => 1 - 1 / Math.sqrt(a * Math.abs(z) + 1),
    },
    {
      key: "obliquestepfun",
      label: "obliquestepfun",
      color: "#ff7f00",
      formula: String.raw`y = \begin{cases} |z|/x & |z| < x \\ 1 & |z| \geq x \end{cases}`,
      params: [{ name: "x", min: 0.1, max: 5, step: 0.01, default: 1.0 }],
      eval: (z, { x }) => {
        const az = Math.abs(z);
        return az >= x ? 1 : az / x;
      },
    },
  ];

  // 默认勾选 dsigmoidn 与 tanhabsx（让用户立即看到内容）。
  const DEFAULT_ENABLED = new Set(["dsigmoidn", "tanhabsx"]);

  // x 轴采样：[-5, 5]，200 点。
  const X_MIN = -5;
  const X_MAX = 5;
  const SAMPLES = 200;
  const X_AXIS = Array.from(
    { length: SAMPLES + 1 },
    (_, i) => X_MIN + ((X_MAX - X_MIN) * i) / SAMPLES
  );
</script>
```

- [ ] **Step 2: 在浏览器控制台手动验证函数求值**

打开页面后在 DevTools Console 运行：

```js
FUNCS.find(f => f.key === "dsigmoidn").eval(0, { a: 1 })
// 预期：约 0（z=0 时 tanh²(0) = 0）

FUNCS.find(f => f.key === "tanhabsx").eval(2, { a: 1 })
// 预期：约 0.964（tanh(2) ≈ 0.964）

FUNCS.find(f => f.key === "obliquestepfun").eval(0.5, { x: 1 })
// 预期：0.5

FUNCS.find(f => f.key === "obliquestepfun").eval(2, { x: 1 })
// 预期：1
```

如有偏差超过 0.01，停下来排查公式。

- [ ] **Step 3: 提交**

```bash
git add docs/index.html
git commit -m "新增对比页面 5 个衰减函数定义与采样工具"
```

---

## Task 4: 状态、Chart 初始化与渲染循环

**Files:**
- Modify: `docs/index.html`（填充 `<script id="ui">` 第一段）

- [ ] **Step 1: 替换 `<script id="ui">` 内容**

将 `<script id="ui">...</script>` 整段替换为：

```html
<script id="ui">
  // ---- 状态 ----
  const state = {
    enabled: new Set(DEFAULT_ENABLED),
    // params: { dsigmoidn: { a: 1.0 }, sigmoidabsx: { a: 1.0, b: 1.0 }, ... }
    params: Object.fromEntries(
      FUNCS.map((f) => [
        f.key,
        Object.fromEntries(f.params.map((p) => [p.name, p.default])),
      ])
    ),
  };

  // ---- 计算指定函数的曲线点 ----
  function computeSeries(func) {
    const args = state.params[func.key];
    return X_AXIS.map((z) => func.eval(z, args));
  }

  // ---- Chart.js 初始化 ----
  const ctx = document.getElementById("chart").getContext("2d");
  const chart = new Chart(ctx, {
    type: "line",
    data: {
      labels: X_AXIS.map((z) => z.toFixed(2)),
      datasets: [], // 由 syncDatasets 填充
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false, // 拖动滑条时无动画感更顺滑
      interaction: { mode: "index", intersect: false },
      scales: {
        x: {
          title: { display: true, text: "z（误差）" },
          ticks: {
            maxTicksLimit: 11,
            callback: function (value) {
              const z = X_AXIS[value];
              return Number.isInteger(z) ? z : "";
            },
          },
        },
        y: {
          title: { display: true, text: "y" },
          min: 0,
          max: 1.05,
        },
      },
      plugins: {
        legend: { position: "top" },
        tooltip: {
          callbacks: {
            title: (items) => `z = ${X_AXIS[items[0].dataIndex].toFixed(2)}`,
          },
        },
      },
      elements: {
        point: { radius: 0 }, // 200 点全画点会卡，关掉
        line: { borderWidth: 2, tension: 0 },
      },
    },
  });

  // ---- 同步 chart 数据集与当前 state.enabled ----
  function syncDatasets() {
    chart.data.datasets = FUNCS.filter((f) => state.enabled.has(f.key)).map(
      (f) => ({
        label: f.label,
        data: computeSeries(f),
        borderColor: f.color,
        backgroundColor: f.color,
        pointHoverRadius: 4,
      })
    );
  }

  // ---- rAF 节流的 chart 重绘 ----
  let rafPending = false;
  function scheduleRender() {
    if (rafPending) return;
    rafPending = true;
    requestAnimationFrame(() => {
      rafPending = false;
      syncDatasets();
      chart.update("none");
    });
  }

  // 初始绘制（其余 UI 在后续 task 装配）
  syncDatasets();
  chart.update();
</script>
```

- [ ] **Step 2: 浏览器刷新验证 Chart 初始化**

预期：图表区域显示空坐标系（x ∈ [-5, 5]，y ∈ [0, 1.05]），有"z（误差）"和"y"轴标签，控制台无错。datasets 暂时为空因为 picker UI 还没生成事件，但默认 enabled 集合下 syncDatasets 会填充 dsigmoidn 和 tanhabsx 两条曲线。

> 实际上由于 `syncDatasets()` 在最后调用，应能立即看到红色（dsigmoidn）和绿色（tanhabsx）两条曲线。如果看不到，检查 Console 报错。

- [ ] **Step 3: 提交**

```bash
git add docs/index.html
git commit -m "新增对比页面 Chart.js 初始化与状态管理"
```

---

## Task 5: 函数选择器（复选框）

**Files:**
- Modify: `docs/index.html`（在 `<script id="ui">` 末尾追加）

- [ ] **Step 1: 在 `chart.update();` 之后，`</script>`（id="ui"）之前追加**

```js
  // ---- 渲染函数选择器复选框 ----
  function renderPicker() {
    const picker = document.getElementById("functionPicker");
    picker.innerHTML = "";

    FUNCS.forEach((f) => {
      const label = document.createElement("label");
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = state.enabled.has(f.key);
      cb.addEventListener("change", () => {
        if (cb.checked) state.enabled.add(f.key);
        else state.enabled.delete(f.key);
        renderParamsPanel();
        scheduleRender();
      });

      const swatch = document.createElement("span");
      swatch.className = "swatch";
      swatch.style.background = f.color;

      const text = document.createElement("span");
      text.textContent = f.label;

      label.append(cb, swatch, text);
      picker.append(label);
    });

    const resetBtn = document.createElement("button");
    resetBtn.type = "button";
    resetBtn.textContent = "重置参数";
    resetBtn.addEventListener("click", () => {
      FUNCS.forEach((f) => {
        f.params.forEach((p) => {
          state.params[f.key][p.name] = p.default;
        });
      });
      renderParamsPanel();
      scheduleRender();
    });
    picker.append(resetBtn);
  }

  renderPicker();
```

- [ ] **Step 2: 浏览器刷新验证**

预期：复选框区域出现 5 个带色块的复选框（dsigmoidn 红、sigmoidabsx 蓝、tanhabsx 绿、px1 紫、obliquestepfun 橙），dsigmoidn 与 tanhabsx 默认勾选；右侧有"重置参数"按钮。

测试交互：
- 勾选/取消复选框 → 图表对应曲线增删
- 取消所有 → 图表为空坐标系
- 重置按钮当前还看不到效果（因为参数面板还没做），不报错即可

- [ ] **Step 3: 提交**

```bash
git add docs/index.html
git commit -m "新增对比页面函数选择器复选框"
```

---

## Task 6: 参数面板（动态滑条 + KaTeX 公式）

**Files:**
- Modify: `docs/index.html`（在 `<script id="ui">` 末尾继续追加）

- [ ] **Step 1: 在 `renderPicker();` 之后追加**

```js
  // ---- 渲染参数面板（仅显示已勾选函数） ----
  function renderParamsPanel() {
    const panel = document.getElementById("paramsPanel");
    panel.innerHTML = "";

    FUNCS.filter((f) => state.enabled.has(f.key)).forEach((f) => {
      const block = document.createElement("div");
      block.className = "param-block";

      const title = document.createElement("div");
      title.className = "title";
      const swatch = document.createElement("span");
      swatch.className = "swatch";
      swatch.style.background = f.color;
      const name = document.createElement("span");
      name.textContent = f.label;
      title.append(swatch, name);

      const formula = document.createElement("div");
      formula.className = "formula";
      formula.textContent = `$$${f.formula}$$`;

      block.append(title, formula);

      f.params.forEach((p) => {
        const row = document.createElement("div");
        row.className = "param-row";

        const labelEl = document.createElement("span");
        labelEl.textContent = `${p.name} =`;

        const slider = document.createElement("input");
        slider.type = "range";
        slider.min = p.min;
        slider.max = p.max;
        slider.step = p.step;
        slider.value = state.params[f.key][p.name];

        const valueEl = document.createElement("span");
        valueEl.className = "value";
        valueEl.textContent = Number(slider.value).toFixed(2);

        slider.addEventListener("input", () => {
          const v = Number(slider.value);
          state.params[f.key][p.name] = v;
          valueEl.textContent = v.toFixed(2);
          scheduleRender();
        });

        row.append(labelEl, slider, valueEl);
        block.append(row);
      });

      panel.append(block);
    });

    // KaTeX 渲染本次新增的公式
    if (window.renderMathInElement) {
      window.renderMathInElement(panel, {
        delimiters: [{ left: "$$", right: "$$", display: true }],
        throwOnError: false,
      });
    }
  }

  // 初始渲染参数面板
  // 等 KaTeX 的 auto-render 脚本加载完再渲染（用 defer 后 DOMContentLoaded 已经在我们之前）
  if (window.renderMathInElement) {
    renderParamsPanel();
  } else {
    window.addEventListener("DOMContentLoaded", renderParamsPanel);
  }
```

- [ ] **Step 2: 浏览器刷新验证**

预期：
- 参数面板出现两块：dsigmoidn 块（含一个 a 滑条，值 1.00）和 tanhabsx 块（含一个 a 滑条，值 1.00）
- 公式以 KaTeX 渲染（分式上下分层、`\tanh^2` 上标显示正确）
- 拖动 dsigmoidn 的 a 滑条 → 红色曲线立即变化、滑条右侧数值同步更新
- 勾选 sigmoidabsx → 出现新参数块（含 a、b 两个滑条）+ 蓝色曲线
- 取消 dsigmoidn 勾选 → 红色曲线消失、参数块也消失
- 点"重置参数"按钮 → 所有滑条回到 1.00

性能：拖动滑条时图表流畅，无明显卡顿。

- [ ] **Step 3: 提交**

```bash
git add docs/index.html
git commit -m "新增对比页面动态参数面板与 KaTeX 公式渲染"
```

---

## Task 7: 加在 `obliquestepfun` 的尖锐拐点处理

**Files:**
- Modify: `docs/index.html`（修改 Task 3 中 `FUNCS` 里 `obliquestepfun` 的 eval 或采样策略）

> 200 个等间距点对 `obliquestepfun` 在 |z|=x 拐点附近会产生轻微锯齿。简单做法：让 X_AXIS 在 ±x 处插点。但这会复杂化静态采样。考虑到 YAGNI，只要拐角不丑就保持现状。本任务**先验证一遍是否真的丑**，丑才改。

- [ ] **Step 1: 浏览器查看 obliquestepfun 拐角效果**

打开页面，仅勾选 `obliquestepfun`，把 x 滑到 1.0、2.0、3.0 等位置，观察 |z|=x 处的折角是否锐利。

- [ ] **Step 2: 决策**

如果拐角看起来明显锯齿/圆滑：进入 Step 3 修改采样策略。
如果拐角已经够锐利（200 点在 [-5,5] 上间距 0.05，肉眼可接受）：跳过 Step 3，直接到 Step 4。

- [ ] **Step 3（仅必要时）: 在 `obliquestepfun` 拐点处加密**

如果 Step 2 决策需要改，把 Task 3 中的 `obliquestepfun.eval` 不变，但额外做：把 `chart.data.labels` 与每个数据集的 x 轴解耦——改用 `{x, y}` 散点-折线，并在 syncDatasets 时为 obliquestepfun 注入 ±x 处的两个额外点。

具体改动 `syncDatasets`：

```js
function syncDatasets() {
  chart.data.datasets = FUNCS.filter((f) => state.enabled.has(f.key)).map(
    (f) => {
      let xs = X_AXIS;
      if (f.key === "obliquestepfun") {
        const x = state.params.obliquestepfun.x;
        xs = [...X_AXIS, -x, x].sort((a, b) => a - b);
      }
      const args = state.params[f.key];
      return {
        label: f.label,
        data: xs.map((z) => ({ x: z, y: f.eval(z, args) })),
        borderColor: f.color,
        backgroundColor: f.color,
        pointHoverRadius: 4,
        parsing: false,
      };
    }
  );
}
```

同时把 chart 配置的 x 轴改为 `type: 'linear'` 而非 category：

在 Task 4 的 chart options 中，把 `scales.x` 改为：

```js
x: {
  type: "linear",
  title: { display: true, text: "z（误差）" },
  min: X_MIN,
  max: X_MAX,
  ticks: { stepSize: 1 },
},
```

并删除 `chart.data.labels` 的设置（散点格式不需要）。

同时更新 tooltip 回调（Task 4 中原来用 `X_AXIS[items[0].dataIndex]`，linear 轴下应改为 `items[0].parsed.x`）：

```js
plugins: {
  legend: { position: "top" },
  tooltip: {
    callbacks: {
      title: (items) => `z = ${items[0].parsed.x.toFixed(2)}`,
    },
  },
},
```

- [ ] **Step 4: 提交（如有改动）**

```bash
git add docs/index.html
git commit -m "优化 obliquestepfun 拐点采样以消除锯齿"
```

如果 Step 3 跳过，则本 Task 无 commit。

---

## Task 8: 在 README 顶部加在线对比页面链接

**Files:**
- Modify: `README.md`

- [ ] **Step 1: 读取当前 README**

当前内容：

```markdown
# PID

这是一个嵌入式PID算法库，已经在电源设备上得多哼多验证，使用较为简单。
```

- [ ] **Step 2: 在标题下方插入链接**

把内容改为：

```markdown
# PID

这是一个嵌入式PID算法库，已经在电源设备上得多哼多验证，使用较为简单。

👉 **[在线对比衰减函数](https://misakamikoto128.github.io/PID/)** — 浏览器打开即可叠加 5 个衰减函数曲线、滑动调参对比形状差异。
```

- [ ] **Step 3: 提交**

```bash
git add README.md
git commit -m "README 增加在线衰减函数对比页面链接"
```

---

## Task 9: 端到端验证

**Files:**（仅手动测试）

- [ ] **Step 1: 启动本地服务器**

```bash
cd C:/Users/liuyu/Desktop/WorkPlace/PID
python -m http.server 8000
```

或 PowerShell：
```powershell
python -m http.server 8000
```

打开 `http://localhost:8000/docs/`。

- [ ] **Step 2: 跑通设计文档中的验证清单**

逐项确认（参考 `docs/superpowers/specs/2026-05-09-decay-functions-viewer-design.md` "验证标准"段）：

1. [ ] Chrome/Edge/Firefox 打开无控制台报错
2. [ ] 5 个复选框任意组合勾选/取消都能正确叠加曲线
3. [ ] 每个滑条拖动时图形实时更新（视觉无卡顿）
4. [ ] KaTeX 公式正确渲染（分式分层、`\tanh^2` 上标、`\sqrt`、`\begin{cases}` 显示正常）
5. [ ] 窗口缩到 < 900px 宽时布局自动堆叠，无横向溢出
6. [ ] "重置参数"按钮恢复全部参数到 1.0

任何项失败 → 回到对应任务排查。

- [ ] **Step 3: 不commit（仅验证）**

如有需要的小修补，作为新 commit 提交，标题如：`修复对比页面 XXX`。

---

## Task 10: 推送并提示用户启用 GitHub Pages

**Files:**（无）

- [ ] **Step 1: 推送到远端**

```bash
git push origin master
```

- [ ] **Step 2: 提示用户配置 GitHub Pages**

向用户输出消息：

> 代码已推送。你需要在 GitHub 仓库做一次性设置才能让 README 里的链接生效：
>
> 1. 打开 https://github.com/MisakaMikoto128/PID/settings/pages
> 2. 在 "Build and deployment" → "Source" 选 **Deploy from a branch**
> 3. Branch 选 `master`，目录选 `/docs`，保存
> 4. 等待几分钟，访问 https://misakamikoto128.github.io/PID/ 验证页面可达
>
> 如果嫌默认 jekyll 处理慢，也可以在 `docs/` 下加一个空的 `.nojekyll` 文件跳过 Jekyll，但本项目没用 Jekyll 特性，可不加。

- [ ] **Step 3: 等用户反馈页面是否正常**

如用户反馈链接 404 或样式异常 → 排查 Pages 配置或 CDN integrity hash。

---
