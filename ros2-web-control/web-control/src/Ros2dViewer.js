import React, { useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import ROS2D from 'ros2d';

const Ros2dViewer = ({ ros }) => {
  const viewerRef = useRef(null);

  useEffect(() => {
    if (ros && viewerRef.current) {
      const viewer = new ROS2D.Viewer({
        divID: viewerRef.current.id,
        width: 800,
        height: 600,
      });

      const gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        continuous: true,
      });

      gridClient.on('change', () => {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
      });
    }
  }, [ros]);

  return <div id="ros2d-viewer" ref={viewerRef}></div>;
};

export default Ros2dViewer;
