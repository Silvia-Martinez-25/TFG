//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.YarpControl
{
    [Serializable]
    public class PositionMsg : Message
    {
        public const string k_RosMessageName = "yarp_control_msgs/Position";
        public override string RosMessageName => k_RosMessageName;

        //  Message used to send position commands to one or multiple joints
        //  names: it can contain the names of the joints to send the position command to. If left empty, a position value for all the joints available must be provided
        //  positions: it must contain a position value for each name listed in the names vector or for every joint if the names vector is left empty
        //  ref_velocities: it can contain the velocity value for a specific position command. It must contain the same number of entries as the positions array or be empty. If empty the already set speed values will to be used
        public string[] names;
        public double[] positions;
        public double[] ref_velocities;

        public PositionMsg()
        {
            this.names = new string[0];
            this.positions = new double[0];
            this.ref_velocities = new double[0];
        }

        public PositionMsg(string[] names, double[] positions, double[] ref_velocities)
        {
            this.names = names;
            this.positions = positions;
            this.ref_velocities = ref_velocities;
        }

        public static PositionMsg Deserialize(MessageDeserializer deserializer) => new PositionMsg(deserializer);

        private PositionMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.names, deserializer.ReadLength());
            deserializer.Read(out this.positions, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.ref_velocities, sizeof(double), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.names);
            serializer.Write(this.names);
            serializer.WriteLength(this.positions);
            serializer.Write(this.positions);
            serializer.WriteLength(this.ref_velocities);
            serializer.Write(this.ref_velocities);
        }

        public override string ToString()
        {
            return "PositionMsg: " +
            "\nnames: " + System.String.Join(", ", names.ToList()) +
            "\npositions: " + System.String.Join(", ", positions.ToList()) +
            "\nref_velocities: " + System.String.Join(", ", ref_velocities.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
