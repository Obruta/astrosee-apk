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
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
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
import java.util.List;
import java.util.Locale;

import ff_hw_msgs.PmcCommand;
import ff_msgs.EkfState;
import geometry_msgs.QuaternionStamped;
import geometry_msgs.Vector3Stamped;
import sensor_msgs.CompressedImage;
import std_msgs.Float64;



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

    private Vector3Stamped clientNavigationPoseEstimate;
    private Vector3Stamped clientControlAcceleration;
    private Vector3Stamped clientGuidancePosition;
    private Vector3Stamped clientGuidanceError;



    private final Context context;
    private final Paint paint;

    private ObjectDetector objectDetector;
    private ImageProcessor imageProcessor;
    private boolean saveImages;
    private boolean processImages;

    // CV Topic
    Publisher<std_msgs.String> cvResultsPub;
    private String CV_Results;

    MessageFactory factory;



    public AstroseeNode(Context applicationContext, String dataPath) {
        // /sdcard/data/com.obruta.astrosee

        CV_Results = ""; // define to be empty

        this.dataPath = dataPath;
        this.dockCamDataPath = dataPath + "/delayed/dock_images";
        onStartCompleteFlag = false;

        File directory = new File(dockCamDataPath);
        if (!directory.exists()) {
            directory.mkdirs();
        }

        this.context = applicationContext;
        this.saveImages = false;
        this.processImages = true;


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

        clientNavigationPoseEstimate = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        clientControlAcceleration = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        clientGuidancePosition = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send
        clientGuidanceError = factory.newFromType(Vector3Stamped._TYPE); // Add one of these for each topic to send

        // CV Topic
        cvResultsPub = connectedNode.newPublisher("/cv_results", std_msgs.String._TYPE);



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
                // Image to bitmap
                ChannelBuffer buffer = image.getData();
                byte[] data = buffer.array();
                Bitmap bitmap = BitmapFactory.decodeByteArray(data, buffer.arrayOffset(),
                        buffer.readableBytes());

                // Bitmap to TensorFlow
                TensorImage tensorImage = imageProcessor.process(TensorImage.fromBitmap(bitmap));

                // Processing
                List<Detection> results = objectDetector.detect(tensorImage);
                if(saveImages) {
                    processResults(results, image.getHeader().getSeq(), bitmap);
                } else {
                    processResults(results, image.getHeader().getSeq());
                }
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
            Category category = detection.getCategories().get(0);
            RectF box = detection.getBoundingBox();
            CV_Results = String.format("Detected: %s, Image Sequence: %s, Score: %s, CentreX: %s, CentreY: %s, Height: %s, Width: %s",
                    category.getLabel(), image_seq, category.getScore(), box.centerX(), box.centerY(), box.height(), box.width());
            Log.i(TAG, CV_Results);

            // Save data to a topic
            std_msgs.String cv_results_string = cvResultsPub.newMessage();
            cv_results_string.setData(CV_Results);
            cvResultsPub.publish(cv_results_string); // publish it


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
}
