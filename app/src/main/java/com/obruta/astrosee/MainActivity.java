package com.obruta.astrosee;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.support.annotation.NonNull;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.MediaController;
import android.widget.VideoView;

import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import gov.nasa.arc.astrobee.android.gs.MessageType;

public class MainActivity extends RosActivity implements AstroseeServiceListener {
    // IP Address ROS Master
    private static final URI ROS_MASTER_URI = URI.create("http://llp:11311");
    private static final String TAG = MainActivity.class.getSimpleName();

    private VideoView videoView;
    private StartAstroseeService gsService;
    private boolean isBound = false;
    private Handler handler;
    private AstroseeNode node;

    private ServiceConnection serviceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            StartAstroseeService.LocalBinder binder = (StartAstroseeService.LocalBinder) service;
            gsService = binder.getService();
            gsService.setListener(MainActivity.this);
            isBound = true;
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            isBound = false;
        }
    };

    public MainActivity() {
        super("Astrosee", "Astrosee Activity", ROS_MASTER_URI);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Get rid of the status bar and navigation bar
        View decorView = getWindow().getDecorView();
        int uiOptions = View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION;
        decorView.setSystemUiVisibility(uiOptions);
        // Make sure the screen doesn't time out
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        // Create my video View
        videoView = (VideoView) findViewById(R.id.KirksVideoView);

        // Set a MediaController for playback controls
        MediaController mediaController = new MediaController(this);
        mediaController.setAnchorView(videoView);
        videoView.setMediaController(mediaController);

        // Set the video URI for assets
        String uriPath = "android.resource://com.obruta.astrosee/raw/obruta_astrobee_eyes";
        videoView.setVideoURI(Uri.parse(uriPath));

        // Start playing the video
        Log.d(TAG,"Starting the video");
        videoView.start();

        // Set an OnCompletionListener to restart the video when it finishes
        videoView.setOnCompletionListener(new MediaPlayer.OnCompletionListener() {
            @Override
            public void onCompletion(MediaPlayer mp) {
                // Restart the video
                Log.d(TAG,"Restarting the video");
                videoView.start();
            }
        });

        Intent serviceIntent = new Intent(this, StartAstroseeService.class);
        bindService(serviceIntent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        nodeMainExecutorService.stopSelf();
        if (videoView != null) {
            videoView.stopPlayback();
        }
        if (isBound) {
            unbindService(serviceConnection);
            isBound = false;
        }
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        node = new AstroseeNode(getApplicationContext(), gsService.getGuestScienceDataBasePath());
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic("hlp");
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(node, nodeConfiguration);

        handler = new Handler(getMainLooper(), new Handler.Callback() {
            @Override
            public boolean handleMessage(@NonNull Message message) {
                gsService.sendData(MessageType.JSON, "data", node.getData().toString());

                // Schedule the next execution
                handler.sendEmptyMessageDelayed(0, 1000); // Choose how frequently to send the data
                return true;
            }
        });

        // Start the handler
        handler.sendEmptyMessage(0);
    }

    @Override
    public void onVisionEnable(boolean enable) {
        node.enableImageProcessing(enable);
    }
}
