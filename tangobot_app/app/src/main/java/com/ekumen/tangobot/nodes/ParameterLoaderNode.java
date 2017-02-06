package com.ekumen.tangobot.nodes;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.yaml.snakeyaml.Yaml;

import java.io.InputStream;
import java.util.List;
import java.util.Map;

/**
 * Loads parameters from a Yaml file into ROS Parameter Server.
 *
 * @author lucas@ekumenlabs.com (Lucas Chiesa).
 */
public class ParameterLoaderNode extends AbstractNodeMain {

    public static final String NODE_NAME = "parameter_loader";
    private final Map<String, Object> params;

    public ParameterLoaderNode(InputStream parameterYmlInputStream) {
        this.params = (Map<String, Object>)(new Yaml()).load(parameterYmlInputStream);
    }

    private void addParams(ParameterTree parameterTree, Map<String, Object> params) {
        for(Map.Entry<String, Object> e: params.entrySet()) {
            if(e.getValue() instanceof String) {
                parameterTree.set(e.getKey(), (String)e.getValue());
            } else if(e.getValue() instanceof Integer) {
                parameterTree.set(e.getKey(), (Integer)e.getValue());
            } else if(e.getValue() instanceof Double) {
                parameterTree.set(e.getKey(), (Double)e.getValue());
            } else if(e.getValue() instanceof Map) {
                parameterTree.set(e.getKey(), (Map)e.getValue());
            } else if(e.getValue() instanceof Boolean) {
                parameterTree.set(e.getKey(), (Boolean)e.getValue());
            } else if(e.getValue() instanceof List) {
                parameterTree.set(e.getKey(), (List)e.getValue());
            } else {
                Log.d(NODE_NAME, "I don't know what type parameter " + e.getKey() + " is. Value = " + e.getValue());
                Log.d(NODE_NAME, "Class name is: " + e.getValue().getClass().getName());
            }
        }
    }

    /**
     * @return the name of the Node that will be used if a name was not
     * specified in the Node's associated NodeConfiguration
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        if(params != null) {
            ParameterTree parameterTree = connectedNode.getParameterTree();

            // TODO: For some reason, setting the / param when using a rosjava master doesn't work
            // It does work fine with an external master, and also setting other params of any type
            // if it's not on / for a rosjava master
            // parameterTree.set(GraphName.root(), params);
            // Using an auxiliary function instead
            addParams(parameterTree, params);

            connectedNode.shutdown();
        }
    }
}
