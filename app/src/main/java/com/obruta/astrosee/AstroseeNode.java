package com.obruta.astrosee;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.Log;

import org.jboss.netty.buffer.ChannelBuffer;
import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.tensorflow.lite.support.image.ImageProcessor;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.core.BaseOptions;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

import ff_hw_msgs.PmcCommand;
import ff_msgs.EkfState;
import geometry_msgs.QuaternionStamped;
import geometry_msgs.Vector3;
import geometry_msgs.Vector3Stamped;
import sensor_msgs.CompressedImage;
import std_msgs.Float64;
import std_msgs.Header;
import std_msgs.Int32;




public class AstroseeNode extends AbstractNodeMain {
    private static final String TAG = AstroseeNode.class.getSimpleName();

    private final String dataPath;
    private final String dockCamDataPath;

    private String robotName = "undef";
    private EkfState robotEKF;
    private PmcCommand pmcCommand;

    // Yazan's custom ones to log
    private Vector3Stamped adaptiveGNCctlAcceleration;
    private Vector3Stamped adaptiveGNCctlXmPos;
    private Vector3Stamped adaptiveGNCctlXmVel;
    private Vector3Stamped adaptiveGNCctlXmPosError;
    private Vector3Stamped adaptiveGNCctlXmVelError;
    private Vector3Stamped adaptiveGNCguidancePos;
    private Vector3Stamped adaptiveGNCguidanceError;
    private Vector3Stamped adaptiveControlTorque;
    private QuaternionStamped quaternionError;
    private QuaternionStamped quaternionDesired;
    private Vector3Stamped clientAdaptiveControlTorque;
    private QuaternionStamped clientQuaternionError;
    private QuaternionStamped clientQuaternionDesired;
    private Vector3Stamped adaptiveGNCnavRelativePosition;
    private Vector3Stamped adaptiveGNCnavClientStates;
    private Boolean onStartCompleteFlag;
    private Float64 simulinkTime;
    private Int32 simulinkPhase;

    private Vector3Stamped clientNavigationPoseEstimate;
    private Vector3Stamped clientControlAcceleration;
    private Vector3Stamped clientGuidancePosition;
    private Vector3Stamped clientGuidanceError;



    private final Context context;
    private final Paint paint;

    private ObjectDetector objectDetector;
    private ObjectDetector poseDetector;
    private ImageProcessor imageProcessor;
    private boolean saveImages;
    private boolean processImages;

    // CV Topics
    Publisher<std_msgs.String> cvResultsPub;
    private String CV_Results;
    private String CV_pose_Results;

    Publisher<Vector3Stamped> cvRelPositionPub;
    Publisher<Vector3Stamped> cvRelRodriugesPub;
    Publisher<Vector3Stamped> cvBBcentrePub;
    //private Vector3Stamped cvRelPosition;

    MessageFactory factory;

    // Hard-coding the objectPoints
    Point3[] pointsArray = new Point3[]{
            new Point3(-0.148048, -0.125818, 0.099267),
            new Point3(0.133917, -0.112317, -0.156238),
            new Point3(-0.120165, 0.106573, -0.114485),
            new Point3(0.133836, 0.142346, 0.141878),
            new Point3(-0.137991, -0.152850, -0.155571),
            new Point3(-0.137991, -0.152850, -0.155571),
            new Point3(-0.148946, 0.137467, 0.138683),
            new Point3(-0.140832, -0.147009, 0.142015)};

    // Convert the array to a MatOfPoint3f
    MatOfPoint3f objectPoints = new MatOfPoint3f(pointsArray);


    public AstroseeNode(Context applicationContext, String dataPath) {
        // /sdcard/data/com.obruta.astrosee

        CV_Results = ""; // define to be empty
        CV_pose_Results = ""; // define to be empty

        this.dataPath = dataPath;
        this.dockCamDataPath = dataPath + "/delayed/dock_images";
        onStartCompleteFlag = false;

        File directory = new File(dockCamDataPath);
        if (!directory.exists()) {
            directory.mkdirs();
        }

        this.context = applicationContext;
        this.saveImages = false;
        this.processImages = false;


        paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3);

    }

    public void enableImageSaving(boolean enable) {
        saveImages = enable;
    }

    public void enableImageProcessing(boolean enable) {
        processImages = enable;
    }

    public JSONObject getData() {

        JSONObject data = new JSONObject();
        if (!onStartCompleteFlag){
            return data;
        }
        try {

            String nozzleSideOne = "";
            String nozzleSideTwo = "";
            if (!pmcCommand.getGoals().isEmpty()) {

                String nozzle1 = String.valueOf(pmcCommand.getGoals().get(0).getNozzlePositions().getUnsignedByte(0));
                String nozzle2 = String.valueOf(pmcCommand.getGoals().get(0).getNozzlePositions().getUnsignedByte(1));
                String nozzle3 = String.valueOf(pmcCommand.getGoals().get(0).getNozzlePositions().getUnsignedByte(2));
                String nozzle4 = String.valueOf(pmcCommand.getGoals().get(0).getNozzlePositions().getUnsignedByte(3));
                String nozzle5 = String.valueOf(pmcCommand.getGoals().get(0).getNozzlePositions().getUnsignedByte(4));
                String nozzle6 = String.valueOf(pmcCommand.getGoals().get(0).getNozzlePositions().getUnsignedByte(5));
                String nozzle7 = String.valueOf(pmcCommand.getGoals().get(1).getNozzlePositions().getUnsignedByte(0));
                String nozzle8 = String.valueOf(pmcCommand.getGoals().get(1).getNozzlePositions().getUnsignedByte(1));
                String nozzle9 = String.valueOf(pmcCommand.getGoals().get(1).getNozzlePositions().getUnsignedByte(2));
                String nozzle10 = String.valueOf(pmcCommand.getGoals().get(1).getNozzlePositions().getUnsignedByte(3));
                String nozzle11 = String.valueOf(pmcCommand.getGoals().get(1).getNozzlePositions().getUnsignedByte(4));
                String nozzle12 = String.valueOf(pmcCommand.getGoals().get(1).getNozzlePositions().getUnsignedByte(5));

                nozzleSideOne = "[" + nozzle1 + ", " + nozzle2 + ", " + nozzle3 + ", " + nozzle4 + ", " + nozzle5 + ", " + nozzle6 + "]";
                nozzleSideTwo = "[" + nozzle7 + ", " + nozzle8 + ", " + nozzle9 + ", " + nozzle10 + ", " + nozzle11 + ", " + nozzle12 + "]";
            }

            data.put("robot_name", robotName)
                .put("Time: ", String.format("%.2f", simulinkTime.getData()))
                .put("Phase: ", String.format("%d", simulinkPhase.getData()))
                //.put("Flight Mode: ", String.valueOf(flightMode.getSpeed()))
                .put("EKF Position: ", "[" + String.format("%.4f", robotEKF.getPose().getPosition().getX()) + ", " + String.format("%.4f", robotEKF.getPose().getPosition().getY()) + ", " + String.format("%.4f", robotEKF.getPose().getPosition().getZ()) + "]") // Kirk look up how to concatenate in Java and make this as clean as possible
                .put("EKF Attitude: ", "[" + String.format("%.4f", robotEKF.getPose().getOrientation().getX()) + ", " + String.format("%.4f", robotEKF.getPose().getOrientation().getY()) + ", " + String.format("%.4f", robotEKF.getPose().getOrientation().getZ()) + ", " + String.format("%.4f", robotEKF.getPose().getOrientation().getW()) + "]") // Kirk look up how to concatenate in Java and make this as clean as possible
                .put("Nozzle Positions Side 1", nozzleSideOne)
                .put("Nozzle Positions Side 2", nozzleSideTwo)
                .put("--------------------", "------------------------------------")
                .put("Client KF Position: ", "[" + String.format("%.4f", clientNavigationPoseEstimate.getVector().getX()) + ", " + String.format("%.4f", clientNavigationPoseEstimate.getVector().getY()) + ", " + String.format("%.4f", clientNavigationPoseEstimate.getVector().getZ()) + "]")
                .put("Client Position Error: ", "[" + String.format("%.4f", clientGuidanceError.getVector().getX()) + ", " + String.format("%.4f", clientGuidanceError.getVector().getY()) + ", " + String.format("%.4f", clientGuidanceError.getVector().getZ()) + "]")
                .put("Client Quaternion Error: ", "[" + String.format("%.4f", clientQuaternionError.getQuaternion().getX()) + ", " + String.format("%.4f", clientQuaternionError.getQuaternion().getY()) + ", " + String.format("%.4f", clientQuaternionError.getQuaternion().getZ()) + ", " + String.format("%.4f", clientQuaternionError.getQuaternion().getW()) + "]")
                .put("Client Quaternion Desired", "[" + String.format("%.4f", clientQuaternionDesired.getQuaternion().getX()) + ", " + String.format("%.4f", clientQuaternionDesired.getQuaternion().getY()) + ", " + String.format("%.4f", clientQuaternionDesired.getQuaternion().getZ()) + ", " + String.format("%.4f", clientQuaternionDesired.getQuaternion().getW()) + "]")
                .put("Client Desired Position: ", "[" + String.format("%.4f", clientGuidancePosition.getVector().getX()) + ", " + String.format("%.4f", clientGuidancePosition.getVector().getY()) + ", " + String.format("%.4f", clientGuidancePosition.getVector().getZ()) + "]")
                .put("Client Desired Acceleration: ", "[" + String.format("%.4f", clientControlAcceleration.getVector().getX()) + ", " + String.format("%.4f", clientControlAcceleration.getVector().getY()) + ", " + String.format("%.4f", clientControlAcceleration.getVector().getZ()) + "]")
                .put("Client Control Torque", "[" + String.format("%.4f", clientAdaptiveControlTorque.getVector().getX()) + ", " + String.format("%.4f", clientAdaptiveControlTorque.getVector().getY()) + ", " + String.format("%.4f", clientAdaptiveControlTorque.getVector().getZ()) + "]")
                .put("-------------------", "-----------------------------------")
                .put("Servicer Xm Position Error", "[" + String.format("%.4f", adaptiveGNCctlXmPosError.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCctlXmPosError.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCctlXmPosError.getVector().getZ()) + "]")
                .put("Servicer Guided Position Error", "[" + String.format("%.4f", adaptiveGNCguidanceError.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCguidanceError.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCguidanceError.getVector().getZ()) + "]")
                .put("Servicer Xm Position", "[" + String.format("%.4f", adaptiveGNCctlXmPos.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCctlXmPos.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCctlXmPos.getVector().getZ()) + "]")
                .put("Servicer Guided Position", "[" + String.format("%.4f", adaptiveGNCguidancePos.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCguidancePos.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCguidancePos.getVector().getZ()) + "]")
                .put("Servicer Xm Velocity", "[" + String.format("%.4f", adaptiveGNCctlXmVel.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCctlXmVel.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCctlXmVel.getVector().getZ()) + "]")
                .put("Servicer Xm Velocity Error", "[" + String.format("%.4f", adaptiveGNCctlXmVelError.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCctlXmVelError.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCctlXmVelError.getVector().getZ()) + "]")
                .put("Servicer Control Acceleration", "[" + String.format("%.4f", adaptiveGNCctlAcceleration.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCctlAcceleration.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCctlAcceleration.getVector().getZ()) + "]")
                .put("Servicer Quaternion Desired", "[" + String.format("%.4f", quaternionDesired.getQuaternion().getX()) + ", " + String.format("%.4f", quaternionDesired.getQuaternion().getY()) + ", " + String.format("%.4f", quaternionDesired.getQuaternion().getZ()) + ", " + String.format("%.4f", quaternionDesired.getQuaternion().getW()) + "]")
                .put("Servicer Quaternion Error", "[" + String.format("%.4f", quaternionError.getQuaternion().getX()) + ", " + String.format("%.4f", quaternionError.getQuaternion().getY()) + ", " + String.format("%.4f", quaternionError.getQuaternion().getZ()) + ", " + String.format("%.4f", quaternionError.getQuaternion().getW()) + "]")
                .put("Servicer Control Torque", "[" + String.format("%.4f", adaptiveControlTorque.getVector().getX()) + ", " + String.format("%.4f", adaptiveControlTorque.getVector().getY()) + ", " + String.format("%.4f", adaptiveControlTorque.getVector().getZ()) + "]")
                .put("Servicer Pos Wrt Client", "[" + String.format("%.4f", adaptiveGNCnavRelativePosition.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCnavRelativePosition.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCnavRelativePosition.getVector().getZ()) + "]")
                .put("Servicer's Recv'd Abs Client Position", "[" + String.format("%.4f", adaptiveGNCnavClientStates.getVector().getX()) + ", " + String.format("%.4f", adaptiveGNCnavClientStates.getVector().getY()) + ", " + String.format("%.4f", adaptiveGNCnavClientStates.getVector().getZ()) + "]")
                .put("Object Detection Results: ", CV_Results)
                .put("Pose Detection Results: ", CV_pose_Results)
                ;


            // More stuff
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return data;
    }



    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("astrosee_hlp_node_w_image_processing");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        factory = connectedNode.getTopicMessageFactory();
        robotEKF = factory.newFromType(EkfState._TYPE);
        pmcCommand = factory.newFromType(PmcCommand._TYPE);
        adaptiveGNCctlAcceleration = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCctlXmPos = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCctlXmVel = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCctlXmPosError = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCctlXmVelError = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCguidancePos = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCguidanceError = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveControlTorque = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        quaternionError = factory.newFromType(QuaternionStamped._TYPE); // Add one of these for each topic to send
        quaternionDesired = factory.newFromType(QuaternionStamped._TYPE); // Add one of these for each topic to send
        clientAdaptiveControlTorque = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        clientQuaternionError = factory.newFromType(QuaternionStamped._TYPE); // Add one of these for each topic to send
        clientQuaternionDesired = factory.newFromType(QuaternionStamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCnavRelativePosition = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        adaptiveGNCnavClientStates = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        simulinkTime = factory.newFromType(Float64._TYPE);
        simulinkPhase = factory.newFromType(Int32._TYPE);


        clientNavigationPoseEstimate = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        clientControlAcceleration = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        clientGuidancePosition = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        clientGuidanceError = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send

        // CV Topics
        cvResultsPub = connectedNode.newPublisher("/cv_results", std_msgs.String._TYPE);
        cvRelPositionPub = connectedNode.newPublisher("/cv/rel_position", Vector3Stamped._TYPE);
        cvRelRodriugesPub = connectedNode.newPublisher("/cv/rel_mrp", Vector3Stamped._TYPE);
        cvBBcentrePub = connectedNode.newPublisher("/cv/bb_centre", Vector3Stamped._TYPE);

        Subscriber<std_msgs.String> robotNameSub = connectedNode.newSubscriber("/robot_name",
                std_msgs.String._TYPE);
        robotNameSub.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String string) {
                robotName = string.getData();
            }
        });

        Subscriber<EkfState> ekfStateSub = connectedNode.newSubscriber("/gnc/ekf", EkfState._TYPE);
        ekfStateSub.addMessageListener((new MessageListener<EkfState>() {
            @Override
            public void onNewMessage(EkfState ekfState) {
                robotEKF = ekfState;
            }
        }));

        Subscriber<PmcCommand> hardwarePMCcommand = connectedNode.newSubscriber("/hw/pmc/command", PmcCommand._TYPE);
        hardwarePMCcommand.addMessageListener((new MessageListener<PmcCommand>() {
            @Override
            public void onNewMessage(PmcCommand pmcCommandIncoming) {
                pmcCommand = pmcCommandIncoming;
            }
        }));


        // CUSTOM TOPICS RECEIVED FOR GDS
        // Servicer Position Control Topics
        Subscriber<Vector3Stamped> adaptiveGNCctlAccelerationSub = connectedNode.newSubscriber("/adaptive_gnc/ctl/acceleration", Vector3Stamped._TYPE);
        adaptiveGNCctlAccelerationSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCctlAcceleration = vector3Stamped;
            }
        }));

        // Maybe exclude this one
        Subscriber<Vector3Stamped> adaptiveGNCctlXmPosSub = connectedNode.newSubscriber("/adaptive_gnc/ctl/xm/pos", Vector3Stamped._TYPE);
        adaptiveGNCctlXmPosSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCctlXmPos = vector3Stamped;
            }
        }));

        // Maybe exclude this one
        Subscriber<Vector3Stamped> adaptiveGNCctlXmVelSub = connectedNode.newSubscriber("/adaptive_gnc/ctl/xm/vel", Vector3Stamped._TYPE);
        adaptiveGNCctlXmVelSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCctlXmVel = vector3Stamped;
            }
        }));

        Subscriber<Vector3Stamped> adaptiveGNCctlXmPosErrorSub = connectedNode.newSubscriber("/adaptive_gnc/ctl/xm/pos_error", Vector3Stamped._TYPE);
        adaptiveGNCctlXmPosErrorSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCctlXmPosError = vector3Stamped;
            }
        }));

        Subscriber<Vector3Stamped> adaptiveGNCctlXmVelErrorSub = connectedNode.newSubscriber("/adaptive_gnc/ctl/xm/vel_error", Vector3Stamped._TYPE);
        adaptiveGNCctlXmVelErrorSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCctlXmVelError = vector3Stamped;
            }
        }));

        Subscriber<Vector3Stamped> adaptiveGNCguidancePosSub = connectedNode.newSubscriber("/adaptive_gnc/guidance/pos", Vector3Stamped._TYPE);
        adaptiveGNCguidancePosSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCguidancePos = vector3Stamped;
            }
        }));

        Subscriber<Vector3Stamped> adaptiveGNCguidanceErrorSub = connectedNode.newSubscriber("/adaptive_gnc/guidance/error", Vector3Stamped._TYPE);
        adaptiveGNCguidanceErrorSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCguidanceError = vector3Stamped;
            }
        }));
        // Done Servicer Position Control Topics

        // Servicer Attitude Control Topics
        Subscriber<Vector3Stamped> adaptiveControlTorqueSub = connectedNode.newSubscriber("/attitude_control/torque", Vector3Stamped._TYPE);
        adaptiveControlTorqueSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveControlTorque = vector3Stamped;
            }
        }));

        Subscriber<QuaternionStamped> quaternionErrorSub = connectedNode.newSubscriber("/attitude_control/q_error", QuaternionStamped._TYPE);
        quaternionErrorSub.addMessageListener((new MessageListener<QuaternionStamped>() {
            @Override
            public void onNewMessage(QuaternionStamped quaternionStamped) {
                quaternionError = quaternionStamped;
            }
        }));

        Subscriber<QuaternionStamped> quaternionDesiredSub = connectedNode.newSubscriber("/attitude_control/qdesired", QuaternionStamped._TYPE);
        quaternionDesiredSub.addMessageListener((new MessageListener<QuaternionStamped>() {
            @Override
            public void onNewMessage(QuaternionStamped quaternionStamped) {
                quaternionDesired = quaternionStamped;
            }
        }));
        // /adaptive_gnc/nav/relative_position (measured relative position)
        Subscriber<Vector3Stamped> adaptiveGNCnavRelativePositionSub = connectedNode.newSubscriber("/adaptive_gnc/nav/relative_position", Vector3Stamped._TYPE);
        adaptiveGNCnavRelativePositionSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCnavRelativePosition = vector3Stamped;
            }
        }));

        // /adaptive_gnc/nav/target_states (target absolute states as seen by the servicer)
        Subscriber<Vector3Stamped> adaptiveGNCnavClientStatesSub = connectedNode.newSubscriber("/adaptive_gnc/nav/target_states", Vector3Stamped._TYPE);
        adaptiveGNCnavClientStatesSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                adaptiveGNCnavClientStates = vector3Stamped;
            }
        }));
        // Done Servicer Attitude Control Topics


        // Client Attitude Control Topics
        Subscriber<Vector3Stamped> clientAdaptiveControlTorqueSub = connectedNode.newSubscriber("client/attitude_control/torque", Vector3Stamped._TYPE);
        clientAdaptiveControlTorqueSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                clientAdaptiveControlTorque = vector3Stamped;
            }
        }));

        Subscriber<QuaternionStamped> clientQuaternionErrorSub = connectedNode.newSubscriber("/client/attitude_control/q_error", QuaternionStamped._TYPE);
        clientQuaternionErrorSub.addMessageListener((new MessageListener<QuaternionStamped>() {
            @Override
            public void onNewMessage(QuaternionStamped quaternionStamped) {
                clientQuaternionError = quaternionStamped;
            }
        }));

        Subscriber<QuaternionStamped> clientQuaternionDesiredSub = connectedNode.newSubscriber("/client/attitude_control/qdesired", QuaternionStamped._TYPE);
        clientQuaternionDesiredSub.addMessageListener((new MessageListener<QuaternionStamped>() {
            @Override
            public void onNewMessage(QuaternionStamped quaternionStamped) {
                clientQuaternionDesired = quaternionStamped;
            }
        }));


        // The time signal
        Subscriber<Float64> simulinkTimeSub = connectedNode.newSubscriber("/simulinkclock", Float64._TYPE);
        simulinkTimeSub.addMessageListener((new MessageListener<Float64>() {
            @Override
            public void onNewMessage(Float64 float64data) {
                simulinkTime = float64data;
            }
        }));

        // The simulink phase
        Subscriber<Int32> simulinkPhaseSub = connectedNode.newSubscriber("/simulinkphase", Int32._TYPE);
        simulinkPhaseSub.addMessageListener((new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 int32data) {
                simulinkPhase = int32data;
            }
        }));

        // More Client States
        Subscriber<Vector3Stamped> clientNavigationPositionEstimateSub = connectedNode.newSubscriber("/client/gnc/nav/position_est", Vector3Stamped._TYPE);
        clientNavigationPositionEstimateSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                clientNavigationPoseEstimate = vector3Stamped;
            }
        }));

        Subscriber<Vector3Stamped> clientControlAccelerationSub = connectedNode.newSubscriber("/client/gnc/ctl/acceleration", Vector3Stamped._TYPE);
        clientControlAccelerationSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                clientControlAcceleration = vector3Stamped;
            }
        }));

        Subscriber<Vector3Stamped> clientGuidancePositionSub = connectedNode.newSubscriber("/client/gnc/guidance/pos", Vector3Stamped._TYPE);
        clientGuidancePositionSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                clientGuidancePosition = vector3Stamped;
            }
        }));

        Subscriber<Vector3Stamped> clientGuidanceErrorSub = connectedNode.newSubscriber("/client/gnc/guidance/error", Vector3Stamped._TYPE);
        clientGuidanceErrorSub.addMessageListener((new MessageListener<Vector3Stamped>() {
            @Override
            public void onNewMessage(Vector3Stamped vector3Stamped) {
                clientGuidanceError = vector3Stamped;
            }
        }));
        // Done logging signals!


        // Build the Object detector neural network!
        BaseOptions baseOptionsBuilder = BaseOptions.builder().setNumThreads(4).build();
        ObjectDetector.ObjectDetectorOptions optionsBuilder = ObjectDetector.ObjectDetectorOptions.builder()
                .setScoreThreshold(0.5f)
                .setMaxResults(1).setBaseOptions(baseOptionsBuilder).build();

        try {
            objectDetector = ObjectDetector.createFromFileAndOptions(context, "dockCamObjectDetector.tflite", optionsBuilder);
        } catch (IOException e) {
            e.printStackTrace();
        }


        // Build the pose detection neural network!
        BaseOptions baseOptionsBuilder_pose = BaseOptions.builder().setNumThreads(4).build();
        ObjectDetector.ObjectDetectorOptions optionsBuilder_pose = ObjectDetector.ObjectDetectorOptions.builder()
                .setScoreThreshold(0.5f)
                .setMaxResults(1).setBaseOptions(baseOptionsBuilder).build();

        try {
            poseDetector = ObjectDetector.createFromFileAndOptions(context, "dockcam_pose.tflite", optionsBuilder_pose);
        } catch (IOException e) {
            e.printStackTrace();
        }

        imageProcessor = new ImageProcessor.Builder().build();



        Subscriber<CompressedImage> dockCamSub = connectedNode.newSubscriber(
                "/mgt/img_sampler/dock_cam/image_record/compressed", CompressedImage._TYPE);
        dockCamSub.addMessageListener(new MessageListener<CompressedImage>() {
            @Override
            public void onNewMessage(CompressedImage image) {
                // Images are mono8 compressed to JPEG
                if (!processImages) {
                    return;
                }
                //String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmssSSS",
                //        Locale.getDefault()).format(new Date());
                //Log.i(TAG, "Processing image at " + timestamp);
                // Image to bitmap
                ChannelBuffer buffer = image.getData();
                byte[] data = buffer.array();
                Bitmap bitmap = BitmapFactory.decodeByteArray(data, buffer.arrayOffset(),
                        buffer.readableBytes());

                // Bitmap to TensorFlow
                TensorImage tensorImage = imageProcessor.process(TensorImage.fromBitmap(bitmap));

                // Processing object
                List<Detection> results = objectDetector.detect(tensorImage);
                if(saveImages) {
                    processResults(results, image.getHeader().getSeq(), bitmap);
                } else {
                    processResults(results, image.getHeader().getSeq());
                }

                // Processing pose
                List<Detection> pose_results = poseDetector.detect(tensorImage);
                processPoseResults(pose_results, image.getHeader().getSeq());

            }
        });

        onStartCompleteFlag = true;
    }

    @Override
    public void onShutdown(Node node) {
        if (!objectDetector.isClosed()) {
            objectDetector.close();
        }
    }

    public void processResults(List<Detection> detections, int image_seq) {
        for (Detection detection : detections) {
            // Object detection results
            Category category = detection.getCategories().get(0);
            RectF box = detection.getBoundingBox();
            CV_Results = String.format("Detected: %s, Image Sequence: %s, Score: %s, CentreX: %s, CentreY: %s, Height: %s, Width: %s",
                    category.getLabel(), image_seq, category.getScore(), box.centerX(), box.centerY(), box.height(), box.width());
            Log.i(TAG, CV_Results);

            // Save data to the topic
            std_msgs.String cv_results_string = cvResultsPub.newMessage();
            cv_results_string.setData(CV_Results);
            cvResultsPub.publish(cv_results_string); // publish it

            // Create Header
            std_msgs.Header hdr = factory.newFromType(Header._TYPE);
            hdr.setStamp(Time.fromMillis(System.currentTimeMillis()));

            // bounding box centre
            Vector3Stamped bb_centre = cvBBcentrePub.newMessage();
            Vector3 vector_2 = factory.newFromType(Vector3._TYPE);
            vector_2.setX(box.centerX());
            vector_2.setY(box.centerY());
            vector_2.setZ(0.0);
            bb_centre.setVector(vector_2);
            bb_centre.setHeader(hdr);
            cvBBcentrePub.publish(bb_centre); // publish it
            Log.i("AstroSee BB Centre", "[" + String.format("%.1f", bb_centre.getVector().getX()) + ", " + String.format("%.1f", bb_centre.getVector().getY()) + ", " + String.format("%.1f", bb_centre.getVector().getZ()) + "]");


        }
    }

    public void processResults(List<Detection> detections, int image_seq, Bitmap bitmap) {
        // Making bitmap mutable
        bitmap = bitmap.copy(Bitmap.Config.ARGB_8888,true);
        // Loading canvas
        Canvas canvas = new Canvas(bitmap);

        for (Detection detection : detections) {
            Category category = detection.getCategories().get(0);
            RectF box = detection.getBoundingBox();
            Log.i(TAG, String.format("Detected: %s, Score: %s, CentreX: %s, CentreY: %s, Height: %s, Width: %s",
                    category.getLabel(), category.getScore(), box.centerX(), box.centerY(), box.height(), box.width()));


            canvas.drawRect(box.left, box.top, box.right, box.bottom, paint);
        }

        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmssSSS",
                Locale.getDefault()).format(new Date());
        String imageFileName = "IMG_" + timestamp + ".jpg";
        File imageFile = new File(dockCamDataPath, imageFileName);
        try {
            FileOutputStream fos = new FileOutputStream(imageFile);
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, fos);
            fos.flush();
            fos.close();
            Log.i(TAG, "Image saved to: " + imageFile.getAbsolutePath());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void processPoseResults(List<Detection> detections, int image_seq) {

        int numClasses = 8; // number of points in ObjectPoints

        // Create a List to store image points directly (replace with ArrayList if needed)
        List<MatOfPoint2f> imagePointsList = new LinkedList<>();


        for (Detection detection : detections) {
            // Object detection results

            FIGURE OUT WHAT detections IS and how to access each class!

            detection.getCategories().get
            Category category = detection.getCategories().;
            int classIndex = category.getIndex(); // Assuming category.getIndex() returns the class order
            // Check if classIndex is within valid range (optional)
            if (classIndex < 0 || classIndex >= numClasses) {
                Log.e(TAG,"Pose detection class index out of range! Class: " + classIndex);
                continue;
            }
            RectF box = detection.getBoundingBox();
            float centreX = box.centerX();
            float centreY = box.centerY();
            float score = category.getScore();
            // Get the list of bounding box centers for the current class
            // Access the corresponding MatOfPoint3f for the class
            MatOfPoint2f imagePoints = imagePointsList.get(classIndex);
            if (imagePoints == null) {
                // If not created yet, create a new MatOfPoint3f for this class
                imagePoints = new MatOfPoint2f();
                imagePointsList.add(classIndex, imagePoints); // Add at specific class index
            }

            // Add the 2D point to the class-wise image points
            imagePoints.push_back(new MatOfPoint2f(new Point(centreX, centreY)));

            // Logging
            Log.i(TAG, String.format("Pose Detection Classes: %s, Score: %s, CentreX: %s, CentreY: %s", classIndex, score, centreX, centreY));
        }

        // Combine all image points into a single MatOfPoint2f
        MatOfPoint2f allImagePoints = new MatOfPoint2f();
        for (MatOfPoint2f points : imagePointsList) {
            allImagePoints.push_back(points);
        }

        // Call solvePnP

        // Camera properties
        // Define camera matrix values
        double fx = 900; // focal length in x-direction
        double fy = 900; // focal length in y-direction
        double cx = 525; // optical center x-coordinate
        double cy = 600; // optical center y-coordinate

        // Define distortion coefficients (assuming a simple model with just k1 and k2)
        double k1 = 0.0; // radial distortion coefficient 1
        double k2 = 0.0; // radial distortion coefficient 2
        double p1 = 0.0; // radial distortion coefficient 2
        double p2 = 0.0; // radial distortion coefficient 2

        // Create cameraMatrix
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(0, 2, cx);
        cameraMatrix.put(1, 2, cy);
        cameraMatrix.put(2, 2, 1.0);

        // Create distCoefficients
        /*
        Mat distCoefficientsMAT = new Mat(1, 3, CvType.CV_64F);
        distCoefficientsMAT.put(0, 0, k1);
        distCoefficientsMAT.put(0, 1, k2);
        distCoefficientsMAT.put(0, 2, p1);
        distCoefficientsMAT.put(0, 3, p2);
        MatOfDouble distCoefficients = new MatOfDouble(distCoefficientsMAT); // Using k1, k2, p1, and p2; others are ignored.
        */
        double[] myCoefficients = new double[] {k1, k2, p1, k2};
        MatOfDouble distCoefficients = new MatOfDouble(myCoefficients); // Using k1, k2, p1, and p2; others are ignored.

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        // FOR TESTING ONLY: ARTIFICALLY DETECTING POINTS TO SEE HOW THE THE EPNP RUNS IF THE NEURAL NETWORK DETECTED THESE POINTS
        /*
        MatOfPoint2f imagePoints2 = new MatOfPoint2f();
        List<MatOfPoint2f> imagePointsList2 = new LinkedList<>();
        imagePointsList2.add(0, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(120, 120)));
        imagePointsList2.add(1, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(100, 180)));
        imagePointsList2.add(2, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(20, 300)));
        imagePointsList2.add(3, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(300, 10)));
        imagePointsList2.add(4, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(200, 200)));
        imagePointsList2.add(5, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(20, 20)));
        imagePointsList2.add(6, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(199, 199)));
        imagePointsList2.add(7, imagePoints2); // Add at specific class index
        imagePoints2.push_back(new MatOfPoint2f(new Point(76, 75)));

        // Combine all image points into a single MatOfPoint2f
        MatOfPoint2f allImagePoints2 = new MatOfPoint2f();
            for (MatOfPoint2f points : imagePointsList2) {
            allImagePoints2.push_back(points);
        }
        */

        // Hard-coding the imagePoints
        Point[] pointsArray = new Point[]{
                new Point(120, 120),
                new Point(100, 130),
                new Point(20, 120),
                new Point(120, 250),
                new Point(199, 120),
                new Point(0, 0),
                new Point(20, 10),
                new Point(200, 0)};

        // Convert the array to a MatOfPoint3f
        MatOfPoint2f imagePoints = new MatOfPoint2f(pointsArray);

        if (imagePoints.rows() < 8) {
            Log.i(TAG, String.format("Skipping EPNP due to too few imagePoints. Num points: %d and Object Points: %d", imagePoints.rows(), objectPoints.rows()));
            return;
        }

        Log.i(TAG,String.format("Running EPNP with NumImagePoints: %d and ObjectPoints: %d", imagePoints.rows(), objectPoints.rows()));
        Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoefficients, rvec, tvec, false, Calib3d.SOLVEPNP_EPNP); // default is SOLVEPNP_ITERATIVE (requires 3 image points) unless otherwise specified

        // Now rvec and tvec contain the rotation and translation vectors, respectively
        // Pose detection results
        // Relative position
        Vector3Stamped cv_rel_position = cvRelPositionPub.newMessage();
        Vector3 vector_1 = factory.newFromType(Vector3._TYPE);
        vector_1.setX(tvec.get(0,0)[0]);
        vector_1.setY(tvec.get(1,0)[0]);
        vector_1.setZ(tvec.get(2,0)[0]);
        cv_rel_position.setVector(vector_1);
        // Create Header
        std_msgs.Header hdr = factory.newFromType(Header._TYPE);
        hdr.setStamp(Time.fromMillis(System.currentTimeMillis()));
        cv_rel_position.setHeader(hdr);
        // Publish to the topic
        cvRelPositionPub.publish(cv_rel_position);
        Log.i("AstroSee relative position", "[" + String.format("%.4f", cv_rel_position.getVector().getX()) + ", " + String.format("%.4f", cv_rel_position.getVector().getY()) + ", " + String.format("%.4f", cv_rel_position.getVector().getZ()) + "]");

        // Relative orientation
        Vector3Stamped cv_rel_rodriuges = cvRelRodriugesPub.newMessage();
        Vector3 rodriuges_1 = factory.newFromType(Vector3._TYPE);
        rodriuges_1.setX(rvec.get(0,0)[0]);
        rodriuges_1.setY(rvec.get(1,0)[0]);
        rodriuges_1.setZ(rvec.get(2,0)[0]);
        cv_rel_rodriuges.setVector(rodriuges_1);
        cv_rel_rodriuges.setHeader(hdr);
        cvRelRodriugesPub.publish(cv_rel_rodriuges); // publish it
        Log.i("AstroSee relative rodriuges rotation", "[" + String.format("%.4f", cv_rel_rodriuges.getVector().getX()) + ", " + String.format("%.4f", cv_rel_rodriuges.getVector().getY()) + ", " + String.format("%.4f", cv_rel_rodriuges.getVector().getZ()) + "]");

        CV_pose_Results = String.format("Pose detection results. Relative position: [%s, %s, %s]; Relative attitude: [%s, %s, %s]",
                tvec.get(0,0)[0], tvec.get(1,0)[0], tvec.get(2,0)[0], rvec.get(0,0)[0], rvec.get(1,0)[0], rvec.get(2,0)[0]);
        Log.i(TAG, CV_pose_Results);

        //TODO: Log each individual keypoint (as a string???)


    }
}
