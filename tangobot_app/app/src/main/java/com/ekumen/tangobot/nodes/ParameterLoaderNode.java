package com.ekumen.tangobot.nodes;

import android.content.Context;
import android.util.Log;
import android.util.Pair;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.yaml.snakeyaml.Yaml;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Loads parameters from Yaml files into ROS Parameter Server.
 * A different namespace can be specified for each of the files to load.
 * To use this node, create a NamedResource array with the resources to load,
 * and start it in the standard RosJava way.
 *
 * @author lucas@ekumenlabs.com (Lucas Chiesa).
 * Modified by jubeira@ekumenlabs.com (Juan I. Ubeira)
 */
public class ParameterLoaderNode extends AbstractNodeMain {

    public static final String NODE_NAME = "parameter_loader";
    private final List<Pair<String, LoadedRawResource>> params = new ArrayList<>();

    /**
     * Default constructor
     * @param resources Array of resources with their respective namespace to load.
     * @param context Where to get resources from (e.g. caller Activity).
     */
    public ParameterLoaderNode(NamedResource[] resources, Context context) {
        for(NamedResource nr : resources) {
            addSingleYmlInput(context.getResources().openRawResource(nr.first),
                    nr.second == null ? "" : nr.second);
        }
    }

    private void addSingleYmlInput(InputStream ymlInputStream, String namespace) {
        this.params.add(new Pair<>(namespace, new LoadedRawResource((new Yaml()).load(ymlInputStream))));
    }

    private void addParams(ParameterTree parameterTree, String namespace, Map<String, Object> params) {
        for(Map.Entry<String, Object> e: params.entrySet()) {
            String fullKeyName = namespace + "/" + e.getKey();
            Log.i(NODE_NAME, "Loading parameter " + fullKeyName + " \nValue = " + e.getValue());

            if(e.getValue() instanceof String) {
                parameterTree.set(fullKeyName, (String)e.getValue());
            } else if(e.getValue() instanceof Integer) {
                parameterTree.set(fullKeyName, (Integer)e.getValue());
            } else if(e.getValue() instanceof Double) {
                parameterTree.set(fullKeyName, (Double)e.getValue());
            } else if(e.getValue() instanceof Map) {
                parameterTree.set(fullKeyName, (Map)e.getValue());
            } else if(e.getValue() instanceof Boolean) {
                parameterTree.set(fullKeyName, (Boolean)e.getValue());
            } else if(e.getValue() instanceof List) {
                parameterTree.set(fullKeyName, (List)e.getValue());
            } else {
                Log.d(NODE_NAME, "I don't know what type parameter " + fullKeyName + " is. Value = " + e.getValue());
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
            for (Pair<String, LoadedRawResource> m : params) {
                addParams(parameterTree, m.first, m.second.resource);
            }

            connectedNode.shutdown();
        }
    }

    /**
     * Wraps a generic Pair to represent a resource to be loaded (Integer) in
     * a certain namespace (String).
     * Use it to define which resources to load, specifying the namespace (prefix) for each one.
     */
    public static class NamedResource extends Pair<Integer, String> {
        public NamedResource(Integer i, String s) {
            super(i, s);
        }
    }

    /**
     * Thin wrapper for the object returned by Yaml.load().
     * The object returned by Yaml.load() is a LinkedHashMap<String, Object>; this class is to
     * keep the code simple.
     */
    private class LoadedRawResource {
        public Map<String, Object> resource;

        LoadedRawResource(Object resource) {
            this.resource = (Map<String, Object>) resource;
        }
    }
}
