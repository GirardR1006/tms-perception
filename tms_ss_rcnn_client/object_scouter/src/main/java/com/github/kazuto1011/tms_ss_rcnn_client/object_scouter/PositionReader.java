package com.github.kazuto1011.tms_ss_rcnn_client.object_scouter;

import android.os.Handler;
import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;


import java.util.List;

import geometry_msgs.Vector3;
import tms_msg_db.Tmsdb;
import tms_msg_db.TmsdbGetData;
import tms_msg_db.TmsdbGetDataRequest;
import tms_msg_db.TmsdbGetDataResponse;
import tms_msg_db.TmsdbStamped;

/*
A ROS Node that gets the position information from a topic and send them to Rviz
 */

public class PositionReader extends AbstractNodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ObjectScouter/PositionHandler");
    }

    private String TAG = "PositionHandler";
    private ConnectedNode mConnectedNode;
    private Handler handler;

    public float x_old,y_old,z_old;

    public float lpfilter(float data, float filterVal, float filteredVal){
        filteredVal=(data*(1-filteredVal) + (filteredVal*filterVal));
        return filteredVal;
    }



    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        Subscriber<TmsdbStamped> data_sub = connectedNode.newSubscriber("tms_db_data",TmsdbStamped._TYPE);
        data_sub.addMessageListener(new MessageListener<TmsdbStamped>() {
            @Override
            public void onNewMessage(TmsdbStamped tmsdb) {
                // static tf::TransformBroadcaster brOc; <---- Need to be implemented
                //Oculus or Moverio
                float x  = 0;
                float y  = 0;
                float z  = 1.5f;
                float rr = 0;
                float rp = 0;
                float ry = 0;

                //id of the device used (Moverio or Oculus)
                //Here we are using moverio only, so no need to make a choice.
                int id = 1002;
                /*
                else if (choice == 0x02) { id = 1001; } // Oculus
                else if (choice == 0x04) { id = 1001; } // Oculus DK2
                else std::cerr << "Unexpected choice" << std::endl;
                */

                int msg_size = tmsdb.getTmsdb().size();
//    std::cout << msg_size <<","<< id << std::endl;

                for(int i=0; i<msg_size; i++) {
                    if (tmsdb.getTmsdb().get(i).getId() == id) {
                        //Get position form the data basis
                        x = (float) tmsdb.getTmsdb().get(i).getX() / 1000;
                        y = (float) tmsdb.getTmsdb().get(i).getY() / 1000;
                        z = (float) tmsdb.getTmsdb().get(i).getZ() / 1000;

                        //filter angles' data
                        ry = Math.round(lpfilter(((float) tmsdb.getTmsdb().get(i).getRy()) * 0.9f + (ry * .1f), 0.005f, ry));
                        rp = Math.round(lpfilter(((float) tmsdb.getTmsdb().get(i).getRr()) * 0.9f + (rp * .1f), 0.5f, rp)); // there is a problem I don't understand in the angles equivalence;

                        //reajustment, otherwise, we cannot reach every angle between 0 and 360 degrees
                        ry = 180 * ry / 161;
                        if (ry > 0) {
                            rp = -180 * rp / 80;
                        } else {
                            rp = 180 * rp / 80;
                        }

                        //if Vicon cannot track Oculus' markers anymore. In order to avoid some display glitches

                        if (x == 0 && y == 0 && z == 0) {
                            x = x_old / 1000;
                            y = y_old / 1000;
                            z = z_old / 1000;
                        }

                        //transformation to send to Rviz
                        org.ros.rosjava_geometry.Vector3 translation = new org.ros.rosjava_geometry.Vector3(x,y,z);
                        Quaternion rotation = new Quaternion();


                        Transform transform;
                        void setRPY(const tfScalar& roll, const tfScalar& pitch, const tfScalar& yaw){
                                    tfScalar halfYaw = tfScalar(yaw) * tfScalar(0.5);
                                    tfScalar halfPitch = tfScalar(pitch) * tfScalar(0.5);
                                    tfScalar halfRoll = tfScalar(roll) * tfScalar(0.5);
                                    tfScalar cosYaw = tfCos(halfYaw);
                                    tfScalar sinYaw = tfSin(halfYaw);
                                    tfScalar cosPitch = tfCos(halfPitch);
                                    tfScalar sinPitch = tfSin(halfPitch);
                                    tfScalar cosRoll = tfCos(halfRoll);
                                    tfScalar sinRoll = tfSin(halfRoll);
                                    setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                                            cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                                            cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                                            cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
                                }

                        q.setRPY(-rr, -rp, ry);

                        transform.setRotation(q);

                        brOc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "oculus_plugin"));
                        brOc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "start_position", "oculus_plugin"));

                        ocubar->x = x * 1000;
                        ocubar->y = y * 1000;
                        ocubar->z = z * 1000;
                        ocubar->rr = -rr;
                        ocubar->rp = -rp;
                        ocubar->ry = ry;
                        ocubar->qw = q.getAngle();
                        ocubar->qx = q.getAxis()[0];
                        ocubar->qy = q.getAxis()[1];
                        ocubar->qz = q.getAxis()[2];
                        x_old = ocubar->x;
                        y_old = ocubar->y;
                        z_old = ocubar->z;
                        ocubar->x_old = x_old;
                        ocubar->y_old = y_old;
                        ocubar->z_old = z_old;

                    }
                }
            }

        },100);

    }

}