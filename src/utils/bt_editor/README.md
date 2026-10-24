# 行为树编辑器

用浏览器打开 `bt_editor.html`。React、Babel、Tailwind 和 ROSLIB 从 CDN 加载，需要网络连接。

导入 XML 后，编辑器根据 `main_tree_to_execute` 选择主树；未指定时选择文件内第一棵树。使用“当前树”切换树定义，各树的画布修改会保留。选中当前文件内的 `SubTree` 引用可点击“打开子树”。

`root`、`include` 和 `TreeNodesModel` 不作为行为节点绘制。导出保留根属性、依赖声明、未编辑的树和节点模型，并更新编辑过的树。编辑过的树内部注释和原排版不会保留。外部 XML 依赖仅显示路径、保留声明，不自动读取；引用外部定义的子树会标为未解析。画布仿真不执行子树引用，不可据此判断完整行为树的运行结果。

## XML 回归检查

需要 Node.js 和 jsdom。可将测试依赖安装在临时目录，不改变项目依赖：

```bash
bt_test_dir=$(mktemp -d)
npm install --prefix "$bt_test_dir" jsdom@26
NODE_PATH="$bt_test_dir/node_modules" node src/utils/bt_editor/test_xml.cjs
```

检查真实 `nav_tree.xml` 的导入与导出、多树保留、子树引用、节点模型、属性转义和无效结构拒绝。

画布中拖动背景或连线可以平移；右键单击连线删除连接。普通单击不会删除连线。

交互回归检查另需 React 和 Babel（在 jsdom 中挂载编辑器，模拟文件读取）：

```bash
npm install --prefix "$bt_test_dir" react@18 react-dom@18 @babel/core @babel/preset-react
NODE_PATH="$bt_test_dir/node_modules" node src/utils/bt_editor/test_interaction.cjs
```
