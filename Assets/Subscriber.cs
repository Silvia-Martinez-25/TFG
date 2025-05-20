using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.YarpControl;

public class teoSubscriber : MonoBehaviour
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

    ArticulationBody[] leftArmJoints, rightArmJoints, leftLegJoints, rightLegJoints, headJoints, trunkJoints;

    ROSConnection ros;

    string leftArmTopic = "/teoUnity/leftArm/position_direct";
    string rightArmTopic = "/teoUnity/rightArm/position_direct";
    string leftLegTopic = "/teoUnity/leftLeg/position_direct";
    string rightLegTopic = "/teoUnity/rightLeg/position_direct";
    string headTopic = "/teoUnity/head/position_direct";
    string trunkTopic = "/teoUnity/trunk/position_direct";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        leftArmJoints = LoadJoints(LeftArmLinks);
        rightArmJoints = LoadJoints(RightArmLinks);
        leftLegJoints = LoadJoints(LeftLegLinks);
        rightLegJoints = LoadJoints(RightLegLinks);
        headJoints = LoadJoints(HeadLinks);
        trunkJoints = LoadJoints(TrunkLinks);

        ros.Subscribe<PositionDirectMsg>(leftArmTopic, msg => ApplyJoints(leftArmJoints, msg));
        ros.Subscribe<PositionDirectMsg>(rightArmTopic, msg => ApplyJoints(rightArmJoints, msg));
        ros.Subscribe<PositionDirectMsg>(leftLegTopic, msg => ApplyJoints(leftLegJoints, msg));
        ros.Subscribe<PositionDirectMsg>(rightLegTopic, msg => ApplyJoints(rightLegJoints, msg));
        ros.Subscribe<PositionDirectMsg>(headTopic, msg => ApplyJoints(headJoints, msg));
        ros.Subscribe<PositionDirectMsg>(trunkTopic, msg => ApplyJoints(trunkJoints, msg));
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
                //Debug.Log($"{linkNames[i]} encontrado.");
            }
            else
            {
                Debug.LogError($"{linkNames[i]} no encontrado.");
            }
        }
        return joints;
    }

    void ApplyJoints(ArticulationBody[] joints, PositionDirectMsg msg)
    {
        for (int i = 0; i < joints.Length && i < msg.positions.Length; i++)
        {
            if (joints[i] != null)
            {
                var drive = joints[i].xDrive;
                drive.target = (float)msg.positions[i] * Mathf.Rad2Deg;
                joints[i].xDrive = drive;
                Debug.Log($" {joints[i].name} actualizado a {msg.positions[i]:F4}");
            }
        }
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
}

