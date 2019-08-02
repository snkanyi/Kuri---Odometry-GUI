ros = new ROSLIB.Ros({

    url: 'ws://[enter URLof laptop]:9090' //progress-3 laptop
}); 

ros.on('connection', function() {
    console.log('kineval: roslib: connect to websocket server.');
});

ros.on('error', function(error) {
    console.log('kineval: roslib: error connecting to websocket server', error);
});

ros.on('close', function() {
    console.log('kineval: roslib: connection to websocket server closed.');
});

////////////////////////////////////////////////////////////////
rosCmdVel = new ROSLIB.Topic({
   ros : ros,
   name : '/mobile_base/commands/velocity',
   messageType : 'geometry_msgs/Twist'
});

rosTwistFwd = new ROSLIB.Message({
    linear : {
        x : 0.4,
        y : 0.0,
        z : 0.0
    },
    angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    }
});

rosTwistBwd = new ROSLIB.Message({
    linear : {
        x : -0.2,
        y : 0.0,
        z : 0.0
    },
    angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    }
});

rosTwistRht = new ROSLIB.Message({
    linear : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    },
    angular : {
        x : 0.0,
        y : 0.0,
        z : -.5
    }
});

rosTwistLft = new ROSLIB.Message({
    linear : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    },
    angular : {
        x : 0.0,
        y : 0.0,
        z : 0.5
    }
});
////////////////////////////////////////////////////
listener_Odom = new ROSLIB.Topic ({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
});

var odomPosition = {xPos : 0, yPos : 0, zPos: 0};
listener_Odom.subscribe(function(message) { 
    //console.log(`Received message on ${listener_Odom.name} : ${JSON.stringify(message)}`);
    return odomPosition = {
        xPos : message.pose.pose.position.x,
        yPos : message.pose.pose.position.y,
        //wPos : message.pose.pose.orientation.w,
        zPos : message.pose.pose.orientation.z,
    };
});

listener_laserScan = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
});

var scan_msg = {ang_min : 0, ang_max : 0, angIncrement : 0,  rangeArray : [], rangeMin: 0, rangeMax: 0};

listener_laserScan.subscribe(function(message){
    //console.log("Received scan_msg: \n" + JSON.stringify(message));
    return scan_msg = {
    ang_min : message.angle_min,
    ang_max : message.angle_max,
    angIncrement : message.angle_increment,
    rangeArray : message.ranges,
    rangeMin : message.range_min,
    rangeMax : message.range_max
    };
});

var conversion = { ang_min_degree : 0, ang_max_degree : 0, range_cartesian_min : 0,  range_cartesian_max : 0 };
conversion.range_cartesian_min.x = 0;
conversion.range_cartesian_min.y = 0;
conversion.range_cartesian_max.x = 0;
conversion.range_cartesian_max.y = 0;

function laserConversion(scan_msg){
    var ang_min_degree = scan_msg.ang_min * (180/Math.PI);
    var ang_max_degree = scan_msg.ang_max * (180/Math.PI);
    var range_cartesian_min = {
        x : scan_msg.rangeMin * Math.cos(ang_min_degree),
        y : scan_msg.rangeMin * Math.sin(ang_min_degree)
    }
    var range_cartesian_max = {
        x : scan_msg.rangeMax * Math.cos(ang_min_degree),
        y : scan_msg.rangeMax * Math.sin(ang_min_degree)
    }

    return conversion = {
        ang_min_degree,
        ang_max_degree,
        range_cartesian_min,
        range_cartesian_max
    }
    //console.log ("This is conversion " + JSON.stringify(conversion));
}

let coords = {
                p1x : 50, p1y : 0, 
                p2x : 15, p2y : -25,
                p3x : -15, p3y : -25, 
                p4x : -15, p4y : 25, 
                p5x : 15, p5y : 25
            }

var scan = [];
function arrayScan(scan_msg){
    var theta, sX, sY, sZ;
    temp_scan = [];

    for (var i = 0; i<scan_msg.rangeArray.length; i++){
        theta = scan_msg.ang_min + (i * scan_msg.angIncrement);
        if(scan_msg.rangeArray[i] != null && scan_msg.rangeArray[i] != Infinity) {
            sX = [100*scan_msg.rangeArray[i] * Math.cos(theta)];
            sY = [100*scan_msg.rangeArray[i] * Math.sin(theta)*-1];
            sZ = 1;
            temp_scan.push([sX,sY, sZ]);
        }
    }
    //console.log(`This is my scan matrix: ${JSON.stringify(scan)}`)
    return temp_scan;
}

function scanMatrix(odomPosition){

    var t = Math.asin(odomPosition.zPos)*2;
    let A = [ [Math.cos(t), -Math.sin(t), 100*odomPosition.xPos],
              [Math.sin(t), Math.cos(t), 100*odomPosition.yPos],
              [0,0,1],
            ];
    scan = arrayScan(scan_msg);

    let lasArray = []
    for (var i =0; i< A.length; i++){
        lasArray[i] = [];
        for (var j =0 ; j< scan[0].length; j++){
            lasArray[i][j] = 0;
            for (var y = 0 ; y < A[0].length ; y++){
                lasArray[i][j] += A[i][y] * scan[y][j];
            }
        }
    }
    return lasArray;
}

function matrix(coords, odomPosition, conversion, scan) {
    //console.log(`This is my scan ${JSON.stringify(scan)}`)
    //console.log("These are the results of conversion : " + JSON.stringify(conversion));
    var t = Math.asin(odomPosition.zPos)*2;
    //console.log("t is " + t);
    let A = [ [Math.cos(t), -Math.sin(t), 100*odomPosition.xPos],
              [Math.sin(t), Math.cos(t), 100*odomPosition.yPos],
              [0,0,1],
            ];
    let B =[ [coords.p1x, coords.p2x, coords.p3x, coords.p4x, coords.p5x, conversion.range_cartesian_min.x, conversion.range_cartesian_max.x], 
             [coords.p1y, coords.p2y, coords.p3y, coords.p4y, coords.p5y, conversion.range_cartesian_min.y, conversion.range_cartesian_max.y],
             [1,1,1,1,1,1,1],
           ];
    let C = []
    for (var i =0; i< A.length; i++){
        C[i] = [];
        for (var j =0 ; j< B[0].length; j++){
            C[i][j] = 0;
            for (var y = 0 ; y < A[0].length ; y++){
                C[i][j] += A[i][y] * B[y][j];
            }
        }
    }
    //console.log(JSON.stringify(C));
    tracker.update(C, scan);
    tracker.newPosition(odomPosition);
    return C;
}
