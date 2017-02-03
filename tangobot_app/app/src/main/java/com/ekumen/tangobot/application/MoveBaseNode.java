package com.ekumen.tangobot.application;


import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMain;

public class MoveBaseNode extends NativeNodeMain {
    public static final String NODE_NAME = "move_base";

    public MoveBaseNode() {
        super("move_base_jni");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    protected native void execute(String rosMasterUri, String rosHostname, String nodeName, String[] remappingArguments);

    @Override
    protected native void shutdown();
}
