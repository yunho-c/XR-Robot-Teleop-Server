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
    python visualize_kinematic_tree_optimal.py --focus-category Left_Hand
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
    
    # Filter connections based on available bones
    filtered_connections = []
    for parent, child in connections:
        if available_bones is None or (parent in available_bones and child in available_bones):
            filtered_connections.append((parent, child))
    
    # If focusing on a specific category, filter further
    if focus_category:
        categories = get_bone_categories()
        if focus_category in categories:
            focus_bones = set(categories[focus_category])
            # Include spine bones as they form the root
            focus_bones.update(categories['Spine'])
            filtered_connections = [
                (parent, child) for parent, child in filtered_connections
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
    
    def get_bone_shape(bone: FullBodyBoneId) -> str:
        """Get shape for a bone based on availability."""
        if available_bones is None or bone in available_bones:
            return 'box'
        else:
            return 'box,dashed'  # Dashed for missing bones
    
    # Add nodes and edges
    nodes_added = set()
    for parent, child in filtered_connections:
        # Add parent node
        if parent not in nodes_added:
            dot.node(
                str(parent.value), 
                parent.name.replace('FullBody_', '').replace('_', '\n'),
                fillcolor=get_bone_color(parent),
                shape=get_bone_shape(parent)
            )
            nodes_added.add(parent)
        
        # Add child node
        if child not in nodes_added:
            dot.node(
                str(child.value), 
                child.name.replace('FullBody_', '').replace('_', '\n'),
                fillcolor=get_bone_color(child),
                shape=get_bone_shape(child)
            )
            nodes_added.add(child)
        
        # Add edge
        dot.edge(str(parent.value), str(child.value))
    
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
    """Generate a Mermaid diagram for the kinematic tree."""
    
    # Filter connections
    filtered_connections = []
    for parent, child in connections:
        if available_bones is None or (parent in available_bones and child in available_bones):
            filtered_connections.append((parent, child))
    
    # If focusing on a specific category, filter further
    if focus_category:
        categories = get_bone_categories()
        if focus_category in categories:
            focus_bones = set(categories[focus_category])
            focus_bones.update(categories['Spine'])  # Include spine
            filtered_connections = [
                (parent, child) for parent, child in filtered_connections
                if parent in focus_bones or child in focus_bones
            ]
    
    # Generate Mermaid syntax
    mermaid_lines = [
        f"```mermaid",
        f"graph TD",
        f"    %% {title}",
    ]
    
    # Add connections
    for parent, child in filtered_connections:
        parent_name = parent.name.replace('FullBody_', '')
        child_name = child.name.replace('FullBody_', '')
        mermaid_lines.append(f"    {parent.value}[{parent_name}] --> {child.value}[{child_name}]")
    
    # Add styling for different categories
    categories = get_bone_categories()
    category_styles = {
        'Spine': 'fill:#FF6B6B',
        'Left_Arm': 'fill:#4ECDC4',
        'Right_Arm': 'fill:#45B7D1',
        'Left_Hand': 'fill:#96CEB4',
        'Right_Hand': 'fill:#FFEAA7',
        'Left_Leg': 'fill:#DDA0DD',
        'Right_Leg': 'fill:#98D8C8',
    }
    
    mermaid_lines.append("    %% Styling")
    for category, bone_list in categories.items():
        if category in category_styles:
            style = category_styles[category]
            for bone in bone_list:
                if available_bones is None or bone in available_bones:
                    mermaid_lines.append(f"    style {bone.value} {style}")
    
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
    
    print(f"\nINTERPOLATION PRIORITIES:")
    print("1. Hand metacarpals (missing in VMD)")
    print("2. Foot detail bones (missing in both)")
    print("3. Scapula and twist bones (missing in both)")


if __name__ == "__main__":
    main()