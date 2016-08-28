def USER_HOME = System.getProperty("user.home")
def bySecond = timestamp("yyyy-MM-dd-HH-mm-ss")
def DIR = USER_HOME + "/logs/" + bySecond

appender("STDOUT", ConsoleAppender) {
    encoder(PatternLayoutEncoder) {
        pattern = "%-4relative [%thread] %-5level %logger{35} - %msg %n"
    }
}

appender("FILE", FileAppender) {
    file = "${DIR}/beswarm.log"
    encoder(PatternLayoutEncoder) {
        pattern = "%-4relative [%thread] %-5level %logger{35} - %msg %n"
    }
}

appender("FILE_DRONE_POSE", FileAppender) {
    file = "${DIR}/dronepose.log"
    encoder(PatternLayoutEncoder) {
        pattern = "%msg %n"
    }
}

appender("FILE_DRONE_BODY_VELOCITY", FileAppender) {
    file = "${DIR}/drone_body_velocity.log"
    encoder(PatternLayoutEncoder) {
        pattern = "%msg %n"
    }
}
appender("FILE_DRONE_VELOCITY", FileAppender) {
    file = "${DIR}/dronevelocity.log"
    encoder(PatternLayoutEncoder) {
        pattern = "%msg %n"
    }
}

appender("FILE_RECEIVED_MESSAGES", FileAppender) {
    file = "${DIR}/received_messages.log"
    encoder(PatternLayoutEncoder) {
        pattern = "%msg %n"
    }
}

root(DEBUG, ["STDOUT", "FILE"])
logger("commands.AbstractFollowTrajectory.poselogger", TRACE, ["FILE_DRONE_POSE"], false)
logger("commands.AbstractFollowTrajectory.velocitylogger", TRACE, ["FILE_DRONE_VELOCITY"], false)
logger("services.parrot.ParrotVelocity4dService.vel", TRACE, ["FILE_DRONE_BODY_VELOCITY"], false)
logger("services.ros_subscribers.MessagesSubscriberService", TRACE, ["FILE_RECEIVED_MESSAGES"], false)

// for operational tests
appender("FILE_ARMARKER_VELOCITY", FileAppender) {
    file = "${DIR}/armarkervelocity.log"
    encoder(PatternLayoutEncoder) {
        pattern = "%msg %n"
    }
}
logger("operationaltesting.StateEstimatorOT.velocity.armarker", TRACE,
        ["FILE_ARMARKER_VELOCITY"], false)

appender("FILE_ODOM_VELOCITY", FileAppender) {
    file = "${DIR}/odomvelocity.log"
    encoder(PatternLayoutEncoder) {
        pattern = "%msg %n"
    }
}
logger("operationaltesting.StateEstimatorOT.velocity.odom", TRACE,
        ["FILE_ODOM_VELOCITY"], false)