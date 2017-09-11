/**
 * Created by timadamson on 8/22/17.
 */

Arm = function (ros) {

    // Set up the arm publishers
    var delta = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/delta',
        messageType: 'access_teleop_msgs/DeltaPX'
    });

    var absolute = new ROSLIB.Topic({
       ros: ros,
       name: '/access_teleop/absolute',
       messageType: 'access_teleop_msgs/PX'
    });


    // A function for publishing to /access_teleop/delta
    this.moveArmByDelta = function (deltaX, deltaY, cameraName) {
        var deltaPX = new ROSLIB.Message({
            camera_name: cameraName,
            delta_x: parseInt(deltaX),
            delta_y: parseInt(deltaY)
        });
        delta.publish(deltaPX);
    };

    // A function for publishing to /access_teleop/absolute
    this.moveArmByAbsolute = function (absX, absY, cameraName) {
        var absolutePX = new ROSLIB.Message({
            camera_name: cameraName,
            pixel_x: absX,
            pixel_y: absY
        });
        absolute.publish(absolutePX);
    };

};