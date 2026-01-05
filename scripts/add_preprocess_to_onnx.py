#!/usr/bin/env python3
"""
为 ONNX 模型添加预处理层，让 TensorRT 可以直接接受 uint8 BGR 图像

预处理操作:
1. Cast: uint8 → float32
2. Div: 归一化 /255
3. Slice + Concat: BGR → RGB (反转通道)
4. Transpose: NHWC → NCHW

注意: letterbox resize 仍需在外部完成 (因为需要返回 scale, dx, dy 用于后处理)

使用方法:
    python add_preprocess_to_onnx.py input.onnx output.onnx --input_size 640
"""

import argparse
import onnx
from onnx import helper, TensorProto, numpy_helper
import numpy as np


def add_preprocess(model_path: str, output_path: str, input_size: int = 640):
    """在模型输入端添加预处理层"""

    model = onnx.load(model_path)
    graph = model.graph

    # 获取原始输入信息
    orig_input = graph.input[0]
    orig_input_name = orig_input.name
    print(f"Original input: {orig_input_name}")
    print(f"Original input shape: {[d.dim_value for d in orig_input.type.tensor_type.shape.dim]}")

    # 创建新输入: uint8, NHWC, BGR
    # 形状: [1, input_size, input_size, 3]
    new_input = helper.make_tensor_value_info(
        "image_bgr_uint8",
        TensorProto.UINT8,
        [1, input_size, input_size, 3]
    )

    # 创建常量
    div_const = numpy_helper.from_array(
        np.array([255.0], dtype=np.float32),
        name="div_const"
    )

    # 节点列表 (从输入到输出)
    preprocess_nodes = []

    # 1. Cast: uint8 → float32
    preprocess_nodes.append(helper.make_node(
        "Cast",
        inputs=["image_bgr_uint8"],
        outputs=["image_float"],
        to=TensorProto.FLOAT,
        name="preprocess_cast"
    ))

    # 2. Div: 归一化 /255
    preprocess_nodes.append(helper.make_node(
        "Div",
        inputs=["image_float", "div_const"],
        outputs=["image_norm"],
        name="preprocess_div"
    ))

    # 3. BGR → RGB (Split + Concat 反转通道)
    # Split 成 B, G, R 三个通道
    preprocess_nodes.append(helper.make_node(
        "Split",
        inputs=["image_norm"],
        outputs=["channel_b", "channel_g", "channel_r"],
        axis=3,  # 沿着通道维度分割
        name="preprocess_split"
    ))

    # Concat: R, G, B 顺序
    preprocess_nodes.append(helper.make_node(
        "Concat",
        inputs=["channel_r", "channel_g", "channel_b"],
        outputs=["image_rgb"],
        axis=3,
        name="preprocess_concat"
    ))

    # 4. Transpose: NHWC → NCHW
    preprocess_nodes.append(helper.make_node(
        "Transpose",
        inputs=["image_rgb"],
        outputs=[orig_input_name],  # 连接到原始模型的输入
        perm=[0, 3, 1, 2],
        name="preprocess_transpose"
    ))

    # 在图的开头插入预处理节点
    for i, node in enumerate(preprocess_nodes):
        graph.node.insert(i, node)

    # 添加常量到 initializer
    graph.initializer.append(div_const)

    # 更换输入
    # 删除旧输入
    del graph.input[0]
    # 添加新输入
    graph.input.insert(0, new_input)

    # 检查模型
    onnx.checker.check_model(model)

    # 保存
    onnx.save(model, output_path)
    print(f"Saved preprocessed model to: {output_path}")
    print(f"New input: image_bgr_uint8 [1, {input_size}, {input_size}, 3] uint8")


def main():
    parser = argparse.ArgumentParser(description="Add preprocessing layers to ONNX model")
    parser.add_argument("input", help="Input ONNX model path")
    parser.add_argument("output", help="Output ONNX model path")
    parser.add_argument("--input_size", type=int, default=640, help="Input size (default: 640)")

    args = parser.parse_args()
    add_preprocess(args.input, args.output, args.input_size)


if __name__ == "__main__":
    main()
