//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.YarpControl
{
    [Serializable]
    public class GetPositionResponse : Message
    {
        public const string k_RosMessageName = "yarp_control_msgs/GetPosition";
        public override string RosMessageName => k_RosMessageName;

        //  modes: it will contain the position for the joints specified in "names" or for all them if "names" is empty
        //  response: a brief string used to signal the state of the result of the request
        //  opt_descr: An optional human readable description of the result of the request
        public double[] positions;
        public string response = "NOT_SPECIFIED";
        public string opt_descr;

        public GetPositionResponse()
        {
            this.positions = new double[0];
            this.opt_descr = "";
        }

        public GetPositionResponse(double[] positions, string response, string opt_descr)
        {
            this.positions = positions;
            this.response = response;
            this.opt_descr = opt_descr;
        }

        public static GetPositionResponse Deserialize(MessageDeserializer deserializer) => new GetPositionResponse(deserializer);

        private GetPositionResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.positions, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.response);
            deserializer.Read(out this.opt_descr);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.positions);
            serializer.Write(this.positions);
            serializer.Write(this.response);
            serializer.Write(this.opt_descr);
        }

        public override string ToString()
        {
            return "GetPositionResponse: " +
            "\npositions: " + System.String.Join(", ", positions.ToList()) +
            "\nresponse: " + response.ToString() +
            "\nopt_descr: " + opt_descr.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
