#!/usr/bin/env python3
"""ROS entry point for synchronized camera/LiDAR semantic mapping."""

import rospy

from semantic_mapper_fusion import SemanticMapperFusion
from semantic_mapper_init import SemanticMapperInitialization
from semantic_mapper_pipeline import SemanticMapperPipeline


class SemanticMapper(SemanticMapperInitialization, SemanticMapperPipeline, SemanticMapperFusion):
    """Compose ROS wiring, synchronized processing, and map fusion."""


if __name__ == "__main__":
    rospy.init_node("semantic_mapper")
    try:
        SemanticMapper()
    except Exception as exception:
        rospy.logfatal("[语义分析] 启动失败: %s", exception)
        raise
    rospy.spin()
