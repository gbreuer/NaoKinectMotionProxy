/* * * * * * * * * * * * * *
 *  This file is part of the NaoKinectMotionProxy library.
 *
 *  NaoKinectMotionProxy is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  NaoKinectMotionProxy is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NaoKinectMotionProxy.  If not, see <http://www.gnu.org/licenses/>.
 * * * * * * * * * * * * * */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;
using Microsoft.Kinect;
using Aldebaran.Proxies;
using KinectMotionRecorder;

namespace NaoKinect
{
    public class NaoKinectProxy
    {
        /// <summary>
        /// IP of Nao to be moved.
        /// </summary>
        private String _naoIP = "192.168.1.101";
        /// <summary>
        /// Motionproxy port for nao.
        /// </summary>
        private int _naoPort = 9559;
        /// <summary>
        /// Connection to Nao motion controls
        /// </summary>
        private MotionProxy _naoMotion;

        public NaoKinectProxy(String ip)
            : this(ip, 9559)
        {}

        public NaoKinectProxy(String ip, int port)
        {
            _naoIP = ip;
            _naoPort = port;

            //Initiating connection and balancing Nao
            _naoMotion = new MotionProxy(_naoIP, port);
        }

        /// <summary>
        /// Stiffens joints to make Nao movement possible.
        /// </summary>
        public void prepareForMovement()
        {
            _naoMotion.setStiffnesses("Body", 1.0f);
            try
            {
                _naoMotion.wbGoToBalance("Legs", 1.0f);
            }
            catch (Exception e)
            {
                throw new Exception(e.Message + "\n" + "Check to make sure your Nao is running the latest firmware.");
            }
        }

        /// <summary>
        /// Kills all current movements
        /// </summary>
        public void resetNao()
        {
            _naoMotion.killAll();
            _naoMotion.setStiffnesses("Body", 0.0f);
        }

        /// <summary>
        /// Plays back motion at speed recorded.
        /// </summary>
        /// <param name="motion"></param>
        public void playbackMotion(KinectMotion motion)
        {
            float secs = motion.seconds;
            if (secs == 0)
                secs = motion.keyframes * .2f; //200ms per keyframe

            playbackMotion(convertMotionToAngles(motion), secs);
        }

        /// <summary>
        /// Converts motion to angles needed by Nao.
        /// </summary>
        /// <param name="motion"></param>
        /// <returns></returns>
        public List<List<float>> convertMotionToAngles(KinectMotion motion)
        {
            List<List<float>> returnAngles = new List<List<float>>();
            for (int n = 0; n < 10; n++)//10 for number of joints
            {
                returnAngles.Add(new List<float>());
            }

            foreach (Player player in motion.poses)
            {
                float[] lastPoseAngles = copyAnglesToArray(player);
                for (int i = 0; i < lastPoseAngles.Length; i++)
                {
                    returnAngles[i].Add(lastPoseAngles[i]);
                }
            }
            return returnAngles;
        }

        /// <summary>
        /// Plays back motion recorded using recordMotion() function.
        /// </summary>
        /// <param name="angles">Angles output from recordMotion() function.</param>
        /// <param name="seconds">Amount of time to execute motion in, in full seconds.
        /// Using a time that is too short may throw an exception for moving Nao too fast.</param>
        public void playbackMotion(List<List<float>> angles, float seconds)
        {
            String[] jointNames = {"HeadYaw","HeadPitch","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll",
                                      "RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll"};

            //this must match order set in recordMotion(int time, int interval)/copyAnglesToArray()
            List<String> jointList = jointNames.ToList<String>();

            //the time interval between recorded poses
            float keyFrameTime = seconds/angles[0].Count;

            //calculates first position and tells Nao to assume starting position prior to beginning movement
            //this reduces the possibility that an error will arise from asking the Nao to move too quickly
            float[] startAngles = new float[angles.Count];
            for (int i = 0; i < startAngles.Length; i++)
            {
                startAngles[i] = angles[i][0];
            }
            _naoMotion.angleInterpolationWithSpeed(jointNames, startAngles, 0.5f);

            //Calculates each keyframe time that corresponds to each recorded pose
            List<float> times = new List<float>();
            for (int n = 0; n < angles[0].Count; n++)
            {
                times.Add((n+1) * keyFrameTime);
            }

            //Creates total array of times to be executed by angleInterpolation
            List<List<float>> totalTimes = new List<List<float>>();
            for (int m = 0; m < angles.Count; m++)
            {
                totalTimes.Add(times);
            }

            //Executes movement
            _naoMotion.angleInterpolation(jointList, angles, totalTimes, true);
        }

        /// <summary>
        /// Outputs wanted angles to format easily used by recordMotion()
        /// </summary>
        /// <returns>Array of angles at this time.</returns>
        public static float[] copyAnglesToArray(Player player)
        {
            float[] angles = new float[10];
            //just does upper body for now
            angles[0] = degreeToRadian(player.head.yaw);
            angles[1] = degreeToRadian(player.head.pitch);
            angles[2] = degreeToRadian(player.lShoulder.pitch);
            angles[3] = degreeToRadian(player.lShoulder.roll);
            angles[4] = degreeToRadian(player.lElbow.yaw);
            angles[5] = degreeToRadian(player.lElbow.roll);
            angles[6] = degreeToRadian(player.rShoulder.pitch);
            angles[7] = degreeToRadian(player.rShoulder.roll);
            angles[8] = degreeToRadian(player.rElbow.yaw);
            angles[9] = degreeToRadian(player.rElbow.roll);

            return angles;
        }

        /// <summary>
        /// Converts degrees to radians.
        /// </summary>
        /// <param name="deg">Degree to be converted.</param>
        /// <returns>Radian equivalent of degree input.</returns>
        private static float degreeToRadian(double deg)
        {
            return (float)(deg * Math.PI / 180);
        }
    }
}
