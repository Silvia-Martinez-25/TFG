using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;

public class TeoPublisher : MonoBehaviour
{
    public static readonly string[] LeftArmLinks = {
        "FrontalLeftShoulder",
        "SagittalLeftShoulder",
        "AxialLeftShoulder",
        "FrontalLeftElbow",
        "AxialLeftWrist",
        "FrontalLeftWrist"
    };

    public static readonly string[] RightArmLinks = {
        "FrontalRightShoulder",
        "SagittalRightShoulder",
        "AxialRightShoulder",
        "FrontalRightElbow",
        "AxialRightWrist",
        "FrontalRightWrist"
    };

    public static readonly string[] LeftLegLinks = {
        "AxialLeftHip",
        "SagittalLeftHip",
        "FrontalLeftHip",
        "FrontalLeftKnee",
        "FrontalLeftAnkle",
        "SagittalLeftAnkle"
    };

    public static readonly string[] RightLegLinks = {
        "AxialRightHip",
        "SagittalRightHip",
        "FrontalRightHip",
        "FrontalRightKnee",
        "FrontalRightAnkle",
        "SagittalRightAnkle"
    };

    public static readonly string[] HeadLinks = {
        "AxialNeck",
        "FrontalNeck"
    };

    public static readonly string[] TrunkLinks = {
        "AxialTrunk",
        "FrontalTrunk"
    };

    [SerializeField] GameObject m_TeoRoot;

    ArticulationBody[] leftArmJoints;
    ArticulationBody[] rightArmJoints;
    ArticulationBody[] leftLegJoints;
    ArticulationBody[] rightLegJoints;
    ArticulationBody[] headJoints;
    ArticulationBody[] trunkJoints;

    ROSConnection ros;

    string leftArmTopic = "/teoUnity/leftArm/state";
    string rightArmTopic = "/teoUnity/rightArm/state";
    string leftLegTopic = "/teoUnity/leftLeg/state";
    string rightLegTopic = "/teoUnity/rightLeg/state";
    string headTopic = "/teoUnity/head/state";
    string trunkTopic = "/teoUnity/trunk/state";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        RegisterTopics();

        leftArmJoints = LoadJoints(LeftArmLinks);
        rightArmJoints = LoadJoints(RightArmLinks);
        leftLegJoints = LoadJoints(LeftLegLinks);
        rightLegJoints = LoadJoints(RightLegLinks);
        headJoints = LoadJoints(HeadLinks);
        trunkJoints = LoadJoints(TrunkLinks);

        InvokeRepeating(nameof(PublishTeo), 1f, 0.05f);
    }

    void RegisterTopics()
    {
        ros.RegisterPublisher<JointStateMsg>(leftArmTopic);
        ros.RegisterPublisher<JointStateMsg>(rightArmTopic);
        ros.RegisterPublisher<JointStateMsg>(leftLegTopic);
        ros.RegisterPublisher<JointStateMsg>(rightLegTopic);
        ros.RegisterPublisher<JointStateMsg>(headTopic);
        ros.RegisterPublisher<JointStateMsg>(trunkTopic);

        ros.RegisterPublisher<PoseMsg>("/teoUnity/leftArm/pose");
        ros.RegisterPublisher<PoseMsg>("/teoUnity/rightArm/pose");
        ros.RegisterPublisher<PoseMsg>("/teoUnity/leftLeg/pose");
        ros.RegisterPublisher<PoseMsg>("/teoUnity/rightLeg/pose");
    }

    ArticulationBody[] LoadJoints(string[] linkNames)
    {
        ArticulationBody[] joints = new ArticulationBody[linkNames.Length];
        for (int i = 0; i < linkNames.Length; i++)
        {
            Transform linkTransform = FindDeepChild(m_TeoRoot.transform, linkNames[i]);
            if (linkTransform != null)
            {
                joints[i] = linkTransform.GetComponent<ArticulationBody>();
            }
        }
        return joints;
    }

    void PublishTeo()
    {
        PublishJoints(leftArmTopic, leftArmJoints, LeftArmLinks);
        PublishJoints(rightArmTopic, rightArmJoints, RightArmLinks);
        PublishJoints(leftLegTopic, leftLegJoints, LeftLegLinks);
        PublishJoints(rightLegTopic, rightLegJoints, RightLegLinks);
        PublishJoints(headTopic, headJoints, HeadLinks);
        PublishJoints(trunkTopic, trunkJoints, TrunkLinks);

        // Publicar posiciones de los extremos de manos y pies (wrist_yaw como último eslabón)
        PublishEndEffectorPose("/teoUnity/leftArm/pose", leftArmJoints, LeftArmLinks);
        PublishEndEffectorPose("/teoUnity/rightArm/pose", rightArmJoints, RightArmLinks);
        PublishEndEffectorPose("/teoUnity/leftLeg/pose", leftLegJoints, LeftLegLinks);
        PublishEndEffectorPose("/teoUnity/rightLeg/pose", rightLegJoints, RightLegLinks);
    }

    void PublishJoints(string topic, ArticulationBody[] joints, string[] names)
    {
        var msg = new JointStateMsg { name = new string[joints.Length], position = new double[joints.Length] };

        for (int i = 0; i < joints.Length; i++)
        {
            msg.name[i] = joints[i]?.name ?? "";
            msg.position[i] = joints[i]?.jointPosition[0] ?? 0.0;
        }

        ros.Publish(topic, msg);
    }

    Transform FindDeepChild(Transform parent, string name)
    {
        foreach (Transform child in parent)
        {
            if (child.name == name) return child;
            var result = FindDeepChild(child, name);
            if (result != null) return result;
        }
        return null;
    }

    void PublishEndEffectorPose(string topic, ArticulationBody[] joints, string[] linkNames)
    {
        // Obtener la última articulación (eslabón final)
        Transform endEffector = FindDeepChild(m_TeoRoot.transform, linkNames[linkNames.Length - 1]);
        if (endEffector != null)
        {
            // Crear mensaje con la posición y rotación
            var msg = new PoseMsg();

            // Obtener posición y rotación
            Vector3 position = endEffector.position;
            Quaternion rotation = endEffector.rotation;

            msg.position = new PointMsg(
                x: position.x, y: position.y, z: position.z
            );

            msg.orientation = new QuaternionMsg(
                x: rotation.x, y: rotation.y, z: rotation.z, w: rotation.w   
            );

            // Publicar el mensaje
            ros.Publish(topic, msg);
        }
    }

}

