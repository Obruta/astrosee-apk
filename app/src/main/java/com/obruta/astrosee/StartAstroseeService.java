package com.obruta.astrosee;

import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;

import org.json.JSONException;
import org.json.JSONObject;

import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.android.gs.StartGuestScienceService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class StartAstroseeService extends StartGuestScienceService {
    private static final String TAG = StartAstroseeService.class.getSimpleName();

    // The API implementation
    private ApiCommandImplementation api = null;
    private final IBinder binder = new LocalBinder();

    public class LocalBinder extends Binder {
        StartAstroseeService getService() {
            return StartAstroseeService.this;
        }
    }

    private AstroseeNode astroseeNode;

    // Method to set the AstroseeNode instance
    public void setAstroseeNode(AstroseeNode node) {
        astroseeNode = node;
    }


    @Override
    public IBinder onBind(Intent intent) {
        return binder;
    }

     /**
     * This function is called when the GS manager starts your apk.
     * Put all of your start up code in here.
     */
    @Override
    public void onGuestScienceStart() {
        // Get a unique instance of the Astrobee API in order to command the robot.
        //api = ApiCommandImplementation.getInstance();

        // Start the interface
        Intent intent = new Intent(this, MainActivity.class);
        startActivity(intent);

        // Inform the GS Manager and the GDS that the app has been started.
        sendStarted("info");

        // Start with vision disabled
        astroseeNode.enableImageProcessing(false); // Disable the vision
    }

    /**
     * This function is called when the GS manager stops your apk.
     * Put all of your clean up code in here. You should also call the terminate helper function
     * at the very end of this function.
     */
    @Override
    public void onGuestScienceStop() {
        // Stop the API
        //api.shutdownFactory();

        // Inform the GS manager and the GDS that this app stopped.
        sendStopped("info");

        // Destroy all connection with the GS Manager.
        terminate();
    }

    /**
     * This function is called when the GS manager sends a custom command to your apk.
     * Please handle your commands in this function.
     *
     * @param command
     */
    @Override
    public void onGuestScienceCustomCmd(String command) {
        /* Inform the Guest Science Manager (GSM) and the Ground Data System (GDS)
         * that this app received a command. */
        sendReceivedCustomCommand("info");

        try {
            // Transform the String command into a JSON object so we can read it.
            JSONObject jCommand = new JSONObject(command);

            // Get the name of the command we received. See commands.xml files in res folder.
            String sCommand = jCommand.getString("name");

            // JSON object that will contain the data we will send back to the GSM and GDS
            JSONObject jResult = new JSONObject();

            switch (sCommand) {
                // You may handle your commands here
                case "start_vision":
                    astroseeNode.enableImageProcessing(true); // Enable the vision
                    jResult.put("Summary", new JSONObject()
                            .put("Status", "OK")
                            .put("Message", "Vision Started"));
                    break;
                case "stop_vision":
                    astroseeNode.enableImageProcessing(false); // Disable the vision
                    jResult.put("Summary", new JSONObject()
                            .put("Status", "OK")
                            .put("Message", "Vision Stopped"));
                    break;
                default:
                    // Inform GS Manager and GDS, then stop execution.
                    jResult.put("Summary", new JSONObject()
                        .put("Status", "ERROR")
                        .put("Message", "Unrecognized command"));
            }

            // Send data to the GS manager to be shown on the Ground Data System.
            sendData(MessageType.JSON, "data", jResult.toString());
        } catch (JSONException e) {
            // Send an error message to the GSM and GDS
            sendData(MessageType.JSON, "data", "ERROR parsing JSON");
        } catch (Exception ex) {
            // Send an error message to the GSM and GDS
            sendData(MessageType.JSON, "data", "Unrecognized ERROR");
        }
    }
}
