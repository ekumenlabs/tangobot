/*
 * Copyright (c) 2015, Ernesto Corbellini, Ekumen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Android Sensors Driver nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.github.ekumenlabs.tangobot_app.application;


import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;

import java.util.Vector;

//import org.apache.commons.logging.Log;
//import org.apache.commons.logging.LogFactory;

/**
 * A node that loads parameters into the parameter server and dies.
 * @author ecorbellini@ekumen.com (Ernesto Corbellini)
 */
public class ParameterLoaderNode implements NodeMain
{
  private static final String logTag = "ParameterLoaderNode";
  //private Log log = LogFactory.getLog(ParameterLoaderNode.class);
  
  public GraphName getDefaultNodeName()
  {
    return GraphName.of("parameter_loader_node");
  }
  
  public void onStart(ConnectedNode connectedNode)
  {
    //log.info("ParameterLoaderNode: creating the CancellableLoop...");
    Log.i(logTag, "Loading parameters...");
    ParameterTree parameterTree = connectedNode.getParameterTree();
    configRobotLocalization(parameterTree);        

    /*connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void setup() {
        log.info("Loading parameters...");
        ParameterTree parameterTree = connectedNode.getParameterTree();
        configRobotLocalization(parameterTree);        
      }
      
      @Override
      protected void loop() throws InterruptedException {
        // Do nothing...
        Thread.sleep(1000);
      }
    });*/
  }
  
  /**
   * Load configuration parameters for the robot_localization node.
   * @param paramTree the parameter tree object representing the ROS parameter server.
   */
  private void configRobotLocalization(ParameterTree paramTree)
  {
    boolean imuConfig[] = {false, false, false,
                          true, true, true,
                          false, false, false,
                          false, false, false,
                          true, true, true};
    String nodeName = "/localization_jni";
    
    paramTree.set(nodeName + "/two_d_mode", true);
    paramTree.set(nodeName + "/base_link_frame", "imu");
    
    paramTree.set(nodeName + "/imu0", "/android/imu");        
    paramTree.set(nodeName + "/imu0_config", vectorize(imuConfig));
  }
  
  /**
   * Converts and array to a vector.
   * @param a Array of elements to convert to a vector.
   * @return A vector with a copy of all the elements of the array.
   */
  private Vector vectorize(boolean[] a)
  {
    int i;
    int len = a.length;
    Vector v = new Vector(len);
    
    for (i=0; i<len; i++)
      v.add(a[i]);
      
      return v;
  }
  
  
  //@Override
  public void onError(Node node, Throwable throwable)
  {
  }
  
  //@Override
  public void onShutdown(Node arg0)
  {
  }

  //@Override
  public void onShutdownComplete(Node arg0)
  {
  }
}

