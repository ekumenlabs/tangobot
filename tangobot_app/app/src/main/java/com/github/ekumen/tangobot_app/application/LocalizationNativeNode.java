package com.github.ekumen.tangobot_app.application;

import org.ros.node.NativeNodeMain;
import org.ros.namespace.GraphName;

/**
 * Class to implement a robot_localization native node.
 **/
public class LocalizationNativeNode extends NativeNodeMain {
  private static final String nodeName = "localization_jni";
  
  public LocalizationNativeNode() {
    super(nodeName);
  }

  public LocalizationNativeNode(String[] remappingArguments) {
    super(nodeName, remappingArguments);
  }
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(nodeName);
  }

  @Override
  protected native void execute(String rosMasterUri, String rosHostname, String rosNodeName, String[] remappingArguments);

  @Override
  protected native void shutdown();
}
