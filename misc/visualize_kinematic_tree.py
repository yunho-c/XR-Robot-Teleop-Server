"""
Optimal visualization of kinematic trees using Graphviz and Mermaid.

This script creates clean, hierarchical visualizations of skeleton structures
that are much more readable than NetworkX spring layouts.

Requirements:
    pip install graphviz
    pip install matplotlib

Usage:
    python visualize_kinematic_tree_optimal.py
    python visualize_kinematic_tree_optimal.py --format svg
    python visualize_kinematic_tree_optimal.py --generate-mermaid
    python visualize_kinematic_tree_optimal.py --focus-category C
    python visualize_kinematic_tree_optimal.py --matrix --generate-mermaid

Features:
    - Shows complete kinematic trees with missing bones as dashed nodes
    - Broken connections displayed as dashed/red lines
    - Color-coded by body part categories
    - Detailed analysis of missing bones and broken chains
"""

import argparse
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional
import csv

try:
    import graphviz
    GRAPHVIZ_AVAILABLE = True
except ImportError:
    GRAPHVIZ_AVAILABLE = False
    print("Warning: graphviz not available. Install with: pip install graphviz")

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
)

# Import mappings from the previous script
VMD_TO_FULLBODY_MAPPING = {
    "全ての親": None,
    "センター": FullBodyBoneId.FullBody_Hips,
    "下半身": FullBodyBoneId.FullBody_SpineLower,
    "上半身": FullBodyBoneId.FullBody_SpineMiddle,
    "上半身2": FullBodyBoneId.FullBody_SpineUpper,
    "首": FullBodyBoneId.FullBody_Neck,
    "頭": FullBodyBoneId.FullBody_Head,
    "肩.L": FullBodyBoneId.FullBody_LeftShoulder,
    "腕.L": FullBodyBoneId.FullBody_LeftArmUpper,
    "ひじ.L": FullBodyBoneId.FullBody_LeftArmLower,
    "肩.R": FullBodyBoneId.FullBody_RightShoulder,
    "腕.R": FullBodyBoneId.FullBody_RightArmUpper,
    "ひじ.R": FullBodyBoneId.FullBody_RightArmLower,
    "手首.L": FullBodyBoneId.FullBody_LeftHandWrist,
    "親指０.L": FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    "親指１.L": FullBodyBoneId.FullBody_LeftHandThumbProximal,
    "親指２.L": FullBodyBoneId.FullBody_LeftHandThumbDistal,
    "人指０.L": FullBodyBoneId.FullBody_LeftHandIndexProximal,
    "人指１.L": FullBodyBoneId.FullBody_LeftHandIndexIntermediate,
    "人指２.L": FullBodyBoneId.FullBody_LeftHandIndexDistal,
    "中指０.L": FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    "中指１.L": FullBodyBoneId.FullBody_LeftHandMiddleIntermediate,
    "中指２.L": FullBodyBoneId.FullBody_LeftHandMiddleDistal,
    "薬指０.L": FullBodyBoneId.FullBody_LeftHandRingProximal,
    "薬指１.L": FullBodyBoneId.FullBody_LeftHandRingIntermediate,
    "薬指２.L": FullBodyBoneId.FullBody_LeftHandRingDistal,
    "小指０.L": FullBodyBoneId.FullBody_LeftHandLittleProximal,
    "小指１.L": FullBodyBoneId.FullBody_LeftHandLittleIntermediate,
    "小指２.L": FullBodyBoneId.FullBody_LeftHandLittleDistal,
    "手首.R": FullBodyBoneId.FullBody_RightHandWrist,
    "親指０.R": FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    "親指１.R": FullBodyBoneId.FullBody_RightHandThumbProximal,
    "親指２.R": FullBodyBoneId.FullBody_RightHandThumbDistal,
    "人指０.R": FullBodyBoneId.FullBody_RightHandIndexProximal,
    "人指１.R": FullBodyBoneId.FullBody_RightHandIndexIntermediate,
    "人指２.R": FullBodyBoneId.FullBody_RightHandIndexDistal,
    "中指０.R": FullBodyBoneId.FullBody_RightHandMiddleProximal,
    "中指１.R": FullBodyBoneId.FullBody_RightHandMiddleIntermediate,
    "中指２.R": FullBodyBoneId.FullBody_RightHandMiddleDistal,
    "薬指０.R": FullBodyBoneId.FullBody_RightHandRingProximal,
    "薬指１.R": FullBodyBoneId.FullBody_RightHandRingIntermediate,
    "薬指２.R": FullBodyBoneId.FullBody_RightHandRingDistal,
    "小指０.R": FullBodyBoneId.FullBody_RightHandLittleProximal,
    "小指１.R": FullBodyBoneId.FullBody_RightHandLittleIntermediate,
    "小指２.R": FullBodyBoneId.FullBody_RightHandLittleDistal,
    "足.L": FullBodyBoneId.FullBody_LeftUpperLeg,
    "ひざ.L": FullBodyBoneId.FullBody_LeftLowerLeg,
    "足首.L": FullBodyBoneId.FullBody_LeftFootAnkle,
    "足.R": FullBodyBoneId.FullBody_RightUpperLeg,
    "ひざ.R": FullBodyBoneId.FullBody_RightLowerLeg,
    "足首.R": FullBodyBoneId.FullBody_RightFootAnkle,
}

MIXAMO_TO_FULLBODY_MAPPING = {
    "mixamorig:Hips": FullBodyBoneId.FullBody_Hips,
    "mixamorig:Spine": FullBodyBoneId.FullBody_SpineLower,
    "mixamorig:Spine1": FullBodyBoneId.FullBody_SpineMiddle,
    "mixamorig:Spine2": FullBodyBoneId.FullBody_SpineUpper,
    "mixamorig:Neck": FullBodyBoneId.FullBody_Neck,
    "mixamorig:Head": FullBodyBoneId.FullBody_Head,
    "mixamorig:LeftShoulder": FullBodyBoneId.FullBody_LeftShoulder,
    "mixamorig:LeftArm": FullBodyBoneId.FullBody_LeftArmUpper,
    "mixamorig:LeftForeArm": FullBodyBoneId.FullBody_LeftArmLower,
    "mixamorig:RightShoulder": FullBodyBoneId.FullBody_RightShoulder,
    "mixamorig:RightArm": FullBodyBoneId.FullBody_RightArmUpper,
    "mixamorig:RightForeArm": FullBodyBoneId.FullBody_RightArmLower,
    "mixamorig:LeftHand": FullBodyBoneId.FullBody_LeftHandWrist,
    "mixamorig:LeftHandThumb1": FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    "mixamorig:LeftHandThumb2": FullBodyBoneId.FullBody_LeftHandThumbProximal,
    "mixamorig:LeftHandThumb3": FullBodyBoneId.FullBody_LeftHandThumbDistal,
    "mixamorig:LeftHandThumb4": FullBodyBoneId.FullBody_LeftHandThumbTip,
    "mixamorig:LeftHandIndex1": FullBodyBoneId.FullBody_LeftHandIndexProximal,
    "mixamorig:LeftHandIndex2": FullBodyBoneId.FullBody_LeftHandIndexIntermediate,
    "mixamorig:LeftHandIndex3": FullBodyBoneId.FullBody_LeftHandIndexDistal,
    "mixamorig:LeftHandIndex4": FullBodyBoneId.FullBody_LeftHandIndexTip,
    "mixamorig:LeftHandMiddle1": FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    "mixamorig:LeftHandMiddle2": FullBodyBoneId.FullBody_LeftHandMiddleIntermediate,
    "mixamorig:LeftHandMiddle3": FullBodyBoneId.FullBody_LeftHandMiddleDistal,
    "mixamorig:LeftHandMiddle4": FullBodyBoneId.FullBody_LeftHandMiddleTip,
    "mixamorig:LeftHandRing1": FullBodyBoneId.FullBody_LeftHandRingProximal,
    "mixamorig:LeftHandRing2": FullBodyBoneId.FullBody_LeftHandRingIntermediate,
    "mixamorig:LeftHandRing3": FullBodyBoneId.FullBody_LeftHandRingDistal,
    "mixamorig:LeftHandRing4": FullBodyBoneId.FullBody_LeftHandRingTip,
    "mixamorig:LeftHandPinky1": FullBodyBoneId.FullBody_LeftHandLittleProximal,
    "mixamorig:LeftHandPinky2": FullBodyBoneId.FullBody_LeftHandLittleIntermediate,
    "mixamorig:LeftHandPinky3": FullBodyBoneId.FullBody_LeftHandLittleDistal,
    "mixamorig:LeftHandPinky4": FullBodyBoneId.FullBody_LeftHandLittleTip,
    "mixamorig:RightHand": FullBodyBoneId.FullBody_RightHandWrist,
    "mixamorig:RightHandThumb1": FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    "mixamorig:RightHandThumb2": FullBodyBoneId.FullBody_RightHandThumbProximal,
    "mixamorig:RightHandThumb3": FullBodyBoneId.FullBody_RightHandThumbDistal,
    "mixamorig:RightHandThumb4": FullBodyBoneId.FullBody_RightHandThumbTip,
    "mixamorig:RightHandIndex1": FullBodyBoneId.FullBody_RightHandIndexProximal,
    "mixamorig:RightHandIndex2": FullBodyBoneId.FullBody_RightHandIndexIntermediate,
    "mixamorig:RightHandIndex3": FullBodyBoneId.FullBody_RightHandIndexDistal,
    "mixamorig:RightHandIndex4": FullBodyBoneId.FullBody_RightHandIndexTip,
    "mixamorig:RightHandMiddle1": FullBodyBoneId.FullBody_RightHandMiddleProximal,
    "mixamorig:RightHandMiddle2": FullBodyBoneId.FullBody_RightHandMiddleIntermediate,
    "mixamorig:RightHandMiddle3": FullBodyBoneId.FullBody_RightHandMiddleDistal,
    "mixamorig:RightHandMiddle4": FullBodyBoneId.FullBody_RightHandMiddleTip,
    "mixamorig:RightHandRing1": FullBodyBoneId.FullBody_RightHandRingProximal,
    "mixamorig:RightHandRing2": FullBodyBoneId.FullBody_RightHandRingIntermediate,
    "mixamorig:RightHandRing3": FullBodyBoneId.FullBody_RightHandRingDistal,
    "mixamorig:RightHandRing4": FullBodyBoneId.FullBody_RightHandRingTip,
    "mixamorig:RightHandPinky1": FullBodyBoneId.FullBody_RightHandLittleProximal,
    "mixamorig:RightHandPinky2": FullBodyBoneId.FullBody_RightHandLittleIntermediate,
    "mixamorig:RightHandPinky3": FullBodyBoneId.FullBody_RightHandLittleDistal,
    "mixamorig:RightHandPinky4": FullBodyBoneId.FullBody_RightHandLittleTip,
    "mixamorig:LeftUpLeg": FullBodyBoneId.FullBody_LeftUpperLeg,
    "mixamorig:LeftLeg": FullBodyBoneId.FullBody_LeftLowerLeg,
    "mixamorig:LeftFoot": FullBodyBoneId.FullBody_LeftFootAnkle,
    "mixamorig:LeftToeBase": FullBodyBoneId.FullBody_LeftFootBall,
    "mixamorig:RightUpLeg": FullBodyBoneId.FullBody_RightUpperLeg,
    "mixamorig:RightLeg": FullBodyBoneId.FullBody_RightLowerLeg,
    "mixamorig:RightFoot": FullBodyBoneId.FullBody_RightFootAnkle,
    "mixamorig:RightToeBase": FullBodyBoneId.FullBody_RightFootBall,
}


def get_bone_categories() -> Dict[str, List[FullBodyBoneId]]:
    """Categorize bones by body part."""
    categories = {
        'Spine': [
            FullBodyBoneId.FullBody_Root,
            FullBodyBoneId.FullBody_Hips,
            FullBodyBoneId.FullBody_SpineLower,
            FullBodyBoneId.FullBody_SpineMiddle,
            FullBodyBoneId.FullBody_SpineUpper,
            FullBodyBoneId.FullBody_Chest,
            FullBodyBoneId.FullBody_Neck,
            FullBodyBoneId.FullBody_Head,
        ],
        'Left_Arm': [
            FullBodyBoneId.FullBody_LeftShoulder,
            FullBodyBoneId.FullBody_LeftScapula,
            FullBodyBoneId.FullBody_LeftArmUpper,
            FullBodyBoneId.FullBody_LeftArmLower,
            FullBodyBoneId.FullBody_LeftHandWristTwist,
            FullBodyBoneId.FullBody_LeftHandWrist,
            FullBodyBoneId.FullBody_LeftHandPalm,
        ],
        'Right_Arm': [
            FullBodyBoneId.FullBody_RightShoulder,
            FullBodyBoneId.FullBody_RightScapula,
            FullBodyBoneId.FullBody_RightArmUpper,
            FullBodyBoneId.FullBody_RightArmLower,
            FullBodyBoneId.FullBody_RightHandWristTwist,
            FullBodyBoneId.FullBody_RightHandWrist,
            FullBodyBoneId.FullBody_RightHandPalm,
        ],
        'Left_Hand': [bone for bone in FullBodyBoneId if 'LeftHand' in bone.name and 'Wrist' not in bone.name and 'Palm' not in bone.name],
        'Right_Hand': [bone for bone in FullBodyBoneId if 'RightHand' in bone.name and 'Wrist' not in bone.name and 'Palm' not in bone.name],
        'Left_Leg': [
            FullBodyBoneId.FullBody_LeftUpperLeg,
            FullBodyBoneId.FullBody_LeftLowerLeg,
            FullBodyBoneId.FullBody_LeftFootAnkleTwist,
            FullBodyBoneId.FullBody_LeftFootAnkle,
            FullBodyBoneId.FullBody_LeftFootSubtalar,
            FullBodyBoneId.FullBody_LeftFootTransverse,
            FullBodyBoneId.FullBody_LeftFootBall,
        ],
        'Right_Leg': [
            FullBodyBoneId.FullBody_RightUpperLeg,
            FullBodyBoneId.FullBody_RightLowerLeg,
            FullBodyBoneId.FullBody_RightFootAnkleTwist,
            FullBodyBoneId.FullBody_RightFootAnkle,
            FullBodyBoneId.FullBody_RightFootSubtalar,
            FullBodyBoneId.FullBody_RightFootTransverse,
            FullBodyBoneId.FullBody_RightFootBall,
        ],
    }
    return categories


def get_available_bones(mapping_dict: Dict) -> Set[FullBodyBoneId]:
    """Get available bones from a mapping dictionary."""
    return {bone_id for bone_id in mapping_dict.values() if bone_id is not None}


def create_graphviz_tree(connections: List[Tuple], available_bones: Set[FullBodyBoneId],
                        title: str, output_format: str = 'png', focus_category: Optional[str] = None) -> str:
    """Create a hierarchical tree visualization using Graphviz."""
    if not GRAPHVIZ_AVAILABLE:
        print("Graphviz not available. Skipping tree generation.")
        return ""

    # Keep ALL connections, but categorize them
    all_connections = connections

    # If focusing on a specific category, filter connections
    if focus_category:
        categories = get_bone_categories()
        if focus_category in categories:
            focus_bones = set(categories[focus_category])
            # Include spine bones as they form the root
            focus_bones.update(categories['Spine'])
            all_connections = [
                (parent, child) for parent, child in connections
                if parent in focus_bones or child in focus_bones
            ]

    # Create Graphviz digraph
    dot = graphviz.Digraph(comment=title)
    dot.attr(rankdir='TB')  # Top to bottom layout
    dot.attr('node', shape='box', style='rounded,filled', fontsize='10')
    dot.attr('edge', arrowsize='0.5')

    # Color scheme for different bone categories
    category_colors = {
        'Spine': '#FF6B6B',
        'Left_Arm': '#4ECDC4',
        'Right_Arm': '#45B7D1',
        'Left_Hand': '#96CEB4',
        'Right_Hand': '#FFEAA7',
        'Left_Leg': '#DDA0DD',
        'Right_Leg': '#98D8C8',
    }

    def get_bone_color(bone: FullBodyBoneId) -> str:
        """Get color for a bone based on its category."""
        categories = get_bone_categories()
        for category, bone_list in categories.items():
            if bone in bone_list:
                return category_colors.get(category, '#CCCCCC')
        return '#CCCCCC'

    def get_bone_style(bone: FullBodyBoneId) -> tuple[str, str]:
        """Get style for a bone based on availability."""
        if available_bones is None or bone in available_bones:
            return 'solid', '1.0'  # solid style, full opacity
        else:
            return 'dashed', '0.6'  # dashed style, reduced opacity

    def get_edge_style(parent: FullBodyBoneId, child: FullBodyBoneId) -> tuple[str, str]:
        """Get edge style based on bone availability."""
        parent_available = available_bones is None or parent in available_bones
        child_available = available_bones is None or child in available_bones

        if parent_available and child_available:
            return 'solid', 'black'  # solid line, black
        else:
            return 'dashed', 'red'   # dashed line, red for broken connections

    # Add nodes and edges
    nodes_added = set()
    for parent, child in all_connections:
        # Add parent node
        if parent not in nodes_added:
            style, alpha = get_bone_style(parent)
            color = get_bone_color(parent)
            # Lighten color for missing bones
            if style == 'dashed':
                # Make color lighter by adding transparency effect
                color += '80'  # Add alpha channel

            dot.node(
                str(parent.value),
                parent.name.replace('FullBody_', '').replace('_', '\n'),
                fillcolor=color,
                style=f'rounded,filled,{style}',
                fontcolor='black' if style == 'solid' else 'gray'
            )
            nodes_added.add(parent)

        # Add child node
        if child not in nodes_added:
            style, alpha = get_bone_style(child)
            color = get_bone_color(child)
            # Lighten color for missing bones
            if style == 'dashed':
                color += '80'  # Add alpha channel

            dot.node(
                str(child.value),
                child.name.replace('FullBody_', '').replace('_', '\n'),
                fillcolor=color,
                style=f'rounded,filled,{style}',
                fontcolor='black' if style == 'solid' else 'gray'
            )
            nodes_added.add(child)

        # Add edge with appropriate style
        edge_style, edge_color = get_edge_style(parent, child)
        dot.edge(
            str(parent.value),
            str(child.value),
            style=edge_style,
            color=edge_color,
            penwidth='2' if edge_style == 'solid' else '1'
        )

    # Generate output
    filename = title.lower().replace(' ', '_').replace('/', '_')
    if focus_category:
        filename += f"_{focus_category.lower()}"

    try:
        output_path = dot.render(filename, format=output_format, cleanup=True)
        print(f"Generated {title} tree: {output_path}")
        return output_path
    except Exception as e:
        print(f"Error generating Graphviz tree: {e}")
        return ""


def generate_mermaid_diagram(connections: List[Tuple], available_bones: Set[FullBodyBoneId],
                           title: str, focus_category: Optional[str] = None) -> str:
    """Generate a Mermaid diagram for the kinematic tree with broken connections."""

    # Keep ALL connections
    all_connections = connections

    # If focusing on a specific category, filter connections
    if focus_category:
        categories = get_bone_categories()
        if focus_category in categories:
            focus_bones = set(categories[focus_category])
            focus_bones.update(categories['Spine'])  # Include spine
            all_connections = [
                (parent, child) for parent, child in connections
                if parent in focus_bones or child in focus_bones
            ]

    # Generate Mermaid syntax
    mermaid_lines = [
        f"```mermaid",
        f"graph TD",
        f"    %% {title}",
        f"    %% Solid lines = complete connections, Dotted lines = broken connections",
    ]

    # Add connections with different styles for broken chains
    for parent, child in all_connections:
        parent_name = parent.name.replace('FullBody_', '')
        child_name = child.name.replace('FullBody_', '')

        parent_available = available_bones is None or parent in available_bones
        child_available = available_bones is None or child in available_bones

        if parent_available and child_available:
            # Solid connection
            mermaid_lines.append(f"    {parent.value}[{parent_name}] --> {child.value}[{child_name}]")
        else:
            # Broken connection - use dotted line
            mermaid_lines.append(f"    {parent.value}[{parent_name}] -.-> {child.value}[{child_name}]")

    # Add styling for different categories and availability
    categories = get_bone_categories()
    category_colors = {
        'Spine': '#FF6B6B',
        'Left_Arm': '#4ECDC4',
        'Right_Arm': '#45B7D1',
        'Left_Hand': '#96CEB4',
        'Right_Hand': '#FFEAA7',
        'Left_Leg': '#DDA0DD',
        'Right_Leg': '#98D8C8',
    }

    mermaid_lines.append("    %% Styling - Available bones (solid) vs Missing bones (dashed)")

    # Collect all bones that appear in the connections
    all_bones = set()
    for parent, child in all_connections:
        all_bones.add(parent)
        all_bones.add(child)

    for bone in all_bones:
        # Get bone category color
        bone_color = '#CCCCCC'  # default
        for category, bone_list in categories.items():
            if bone in bone_list and category in category_colors:
                bone_color = category_colors[category]
                break

        # Style based on availability
        if available_bones is None or bone in available_bones:
            # Available bone - solid style
            mermaid_lines.append(f"    style {bone.value} fill:{bone_color},stroke:#333,stroke-width:2px")
        else:
            # Missing bone - dashed style with lighter color
            lighter_color = bone_color + "40"  # Add transparency
            mermaid_lines.append(f"    style {bone.value} fill:{lighter_color},stroke:#999,stroke-width:1px,stroke-dasharray: 5 5")

    mermaid_lines.append("```")

    # Save to file
    filename = f"{title.lower().replace(' ', '_').replace('/', '_')}"
    if focus_category:
        filename += f"_{focus_category.lower()}"
    filename += ".md"

    with open(filename, 'w') as f:
        f.write('\n'.join(mermaid_lines))

    print(f"Generated Mermaid diagram: {filename}")
    return '\n'.join(mermaid_lines)


def create_comparison_matrix():
    """Create a visual comparison matrix of bone availability."""
    openxr_bones = set(FullBodyBoneId)
    vmd_bones = get_available_bones(VMD_TO_FULLBODY_MAPPING)
    fbx_bones = get_available_bones(MIXAMO_TO_FULLBODY_MAPPING)

    categories = get_bone_categories()

    # Create matrix data
    matrix_data = []
    bone_names = []

    for category, bone_list in categories.items():
        for bone in bone_list:
            bone_names.append(f"{category}\n{bone.name.replace('FullBody_', '')}")
            row = [
                1,  # OpenXR (all bones available)
                1 if bone in vmd_bones else 0,  # VMD
                1 if bone in fbx_bones else 0,  # FBX
            ]
            matrix_data.append(row)

    # Create visualization
    fig, ax = plt.subplots(figsize=(8, 20))

    import numpy as np
    matrix = np.array(matrix_data)

    # Create heatmap
    im = ax.imshow(matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=1)

    # Set labels
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels(['OpenXR', 'VMD', 'FBX'])
    ax.set_yticks(range(len(bone_names)))
    ax.set_yticklabels(bone_names, fontsize=8)

    # Add text annotations
    for i in range(len(bone_names)):
        for j in range(3):
            text = '✓' if matrix[i, j] == 1 else '✗'
            color = 'white' if matrix[i, j] == 0 else 'black'
            ax.text(j, i, text, ha='center', va='center', color=color, fontweight='bold')

    ax.set_title('Bone Availability Comparison Matrix', fontsize=14, fontweight='bold', pad=20)

    # Add legend
    legend_elements = [
        mpatches.Patch(color='green', label='Available'),
        mpatches.Patch(color='red', label='Missing')
    ]
    ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.05, 1))

    plt.tight_layout()
    plt.savefig('bone_availability_matrix.png', dpi=300, bbox_inches='tight')
    print("Generated bone availability matrix: bone_availability_matrix.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Optimal kinematic tree visualization")
    parser.add_argument("--format", choices=['png', 'svg', 'pdf'], default='png',
                       help="Output format for Graphviz diagrams")
    parser.add_argument("--generate-mermaid", action="store_true",
                       help="Generate Mermaid diagrams")
    parser.add_argument("--focus-category", choices=[
        'Spine', 'Left_Arm', 'Right_Arm', 'Left_Hand', 'Right_Hand', 'Left_Leg', 'Right_Leg'
    ], help="Focus on a specific body part category")
    parser.add_argument("--matrix", action="store_true",
                       help="Generate bone availability comparison matrix")

    args = parser.parse_args()

    print("Optimal Kinematic Tree Visualization")
    print("=" * 40)

    # Get bone sets
    openxr_bones = set(FullBodyBoneId)
    vmd_bones = get_available_bones(VMD_TO_FULLBODY_MAPPING)
    fbx_bones = get_available_bones(MIXAMO_TO_FULLBODY_MAPPING)

    print(f"OpenXR bones: {len(openxr_bones)}")
    print(f"VMD bones: {len(vmd_bones)} ({len(vmd_bones)/len(openxr_bones)*100:.1f}%)")
    print(f"FBX bones: {len(fbx_bones)} ({len(fbx_bones)/len(openxr_bones)*100:.1f}%)")

    if GRAPHVIZ_AVAILABLE:
        # Generate Graphviz trees
        print("\nGenerating Graphviz tree diagrams...")
        create_graphviz_tree(FULL_BODY_SKELETON_CONNECTIONS, openxr_bones,
                           "OpenXR_Complete", args.format, args.focus_category)
        create_graphviz_tree(FULL_BODY_SKELETON_CONNECTIONS, vmd_bones,
                           "VMD_Available", args.format, args.focus_category)
        create_graphviz_tree(FULL_BODY_SKELETON_CONNECTIONS, fbx_bones,
                           "FBX_Available", args.format, args.focus_category)

    if args.generate_mermaid:
        # Generate Mermaid diagrams
        print("\nGenerating Mermaid diagrams...")
        generate_mermaid_diagram(FULL_BODY_SKELETON_CONNECTIONS, openxr_bones,
                               "OpenXR_Complete", args.focus_category)
        generate_mermaid_diagram(FULL_BODY_SKELETON_CONNECTIONS, vmd_bones,
                               "VMD_Available", args.focus_category)
        generate_mermaid_diagram(FULL_BODY_SKELETON_CONNECTIONS, fbx_bones,
                               "FBX_Available", args.focus_category)

    if args.matrix:
        # Generate comparison matrix
        print("\nGenerating bone availability matrix...")
        create_comparison_matrix()

    # Analysis summary
    print("\n" + "=" * 60)
    print("QUICK ANALYSIS SUMMARY")
    print("=" * 60)

    missing_vmd = openxr_bones - vmd_bones
    missing_fbx = openxr_bones - fbx_bones

    print(f"\nMissing in VMD: {len(missing_vmd)} bones")
    print("Key missing VMD bones:")
    for bone in list(missing_vmd)[:10]:
        print(f"  - {bone.name}")

    print(f"\nMissing in FBX: {len(missing_fbx)} bones")
    print("Key missing FBX bones:")
    for bone in list(missing_fbx)[:10]:
        print(f"  - {bone.name}")

    # Analyze broken connections
    print(f"\nBROKEN KINEMATIC CHAINS:")

    def analyze_broken_connections(bone_set: Set[FullBodyBoneId], format_name: str):
        broken_connections = []
        for parent, child in FULL_BODY_SKELETON_CONNECTIONS:
            parent_available = parent in bone_set
            child_available = child in bone_set
            if not (parent_available and child_available):
                broken_connections.append((parent, child, parent_available, child_available))

        print(f"\n{format_name} broken connections: {len(broken_connections)}")

        # Group by category for better understanding
        hand_breaks = [conn for conn in broken_connections if 'Hand' in conn[0].name or 'Hand' in conn[1].name]
        foot_breaks = [conn for conn in broken_connections if 'Foot' in conn[0].name or 'Foot' in conn[1].name]
        other_breaks = [conn for conn in broken_connections if conn not in hand_breaks and conn not in foot_breaks]

        if hand_breaks:
            print(f"  Hand chain breaks: {len(hand_breaks)}")
            for parent, child, p_avail, c_avail in hand_breaks[:5]:  # Show first 5
                status = f"({parent.name.split('_')[-1]} {'✓' if p_avail else '✗'} → {child.name.split('_')[-1]} {'✓' if c_avail else '✗'})"
                print(f"    {status}")

        if foot_breaks:
            print(f"  Foot chain breaks: {len(foot_breaks)}")

        if other_breaks:
            print(f"  Other breaks: {len(other_breaks)}")

        return broken_connections

    vmd_broken = analyze_broken_connections(vmd_bones, "VMD")
    fbx_broken = analyze_broken_connections(fbx_bones, "FBX")

    print(f"\nVISUALIZATION LEGEND:")
    print("- Solid nodes/lines: Available bones and intact connections")
    print("- Dashed nodes: Missing bones")
    print("- Dashed/dotted lines: Broken connections (one or both bones missing)")
    print("- Red lines (Graphviz): Highlight broken kinematic chains")

    print(f"\nINTERPOLATION PRIORITIES:")
    print("1. Hand metacarpals (missing in VMD) - breaks finger chains")
    print("2. Foot detail bones (missing in both) - breaks foot chains")
    print("3. Scapula and twist bones (missing in both) - affects arm realism")


if __name__ == "__main__":
    main()
