using System;
using System.Collections.Generic;

/* Special thanks to erik.nordeus@gmail.com for his core dubins code originally written
 * in Unity. I converted this to work as a class in C# from that Unity C Script
 *  http://www.habrador.com/about/
 */

namespace AgOpenGPS
{
    //To keep track of the different paths when debugging
    public enum PathType
    { RSR, LSL, RSL, LSR, RLR, LRL }

    //Display the final Dubins Paths
    public class CDubins
    {
        //How far we are driving each update, the accuracy will improve if we lower the driveDistance
        public static readonly double driveDistance = 0.05;

        //The radius the car can turn 360 degrees with
        public static double turningRadius = Properties.Settings.Default.set_youTurnRadius;

        //Position, Heading is in radians
        private vec2 startPos, goalPos;

        private double startHeading, goalHeading;

        private List<OneDubinsPath> pathDataList = new List<OneDubinsPath>();

        private readonly List<vec3> dubinsShortestPathList = new List<vec3>();

        //takes 2 points and headings to create a path - returns list of vec3 points and headings
        public List<vec3> GenerateDubins(vec3 _start, vec3 _goal)
        {
            //positions and heading
            startPos.easting = _start.easting;
            startPos.northing = _start.northing;
            startHeading = _start.heading;

            goalPos.easting = _goal.easting;
            goalPos.northing = _goal.northing;
            goalHeading = _goal.heading;

            //Get all valid Dubins paths
            pathDataList = GetAllDubinsPaths();

            //clear out existing path of vec3 points
            dubinsShortestPathList.Clear();

            if (pathDataList.Count > 0)
            {
                int cnt = pathDataList[0].pathCoordinates.Count;
                if (cnt > 1)
                {
                    //calculate the heading for each point
                    for (int i = 0; i < cnt - 1; i += 5)
                    {
                        vec3 pt = new vec3(pathDataList[0].pathCoordinates[i].easting, pathDataList[0].pathCoordinates[i].northing, 0)
                        {
                            heading = Math.Atan2(pathDataList[0].pathCoordinates[i + 1].easting - pathDataList[0].pathCoordinates[i].easting,
                            pathDataList[0].pathCoordinates[i + 1].northing - pathDataList[0].pathCoordinates[i].northing)
                        };
                        dubinsShortestPathList.Add(pt);
                    }
                }
            }
            return dubinsShortestPathList;
        }

        //The 4 different circles we have that sits to the left/right of the start/goal
        private vec2 startLeftCircle, startRightCircle, goalLeftCircle, goalRightCircle;

        //Get all valid Dubins paths sorted from shortest to longest
        private List<OneDubinsPath> GetAllDubinsPaths()
        {
            pathDataList.Clear();
            PositionLeftRightCircles();
            CalculateDubinsPathsLengths();

            if (pathDataList.Count > 0)
            {
                pathDataList.Sort((x, y) => x.totalLength.CompareTo(y.totalLength));
                GeneratePathCoordinates();
            }
            else
            {
                pathDataList.Clear();
            }

            return pathDataList;
        }

        private void PositionLeftRightCircles()
        {
            goalRightCircle = DubinsMath.GetRightCircleCenterPos(goalPos, goalHeading);
            goalLeftCircle = DubinsMath.GetLeftCircleCenterPos(goalPos, goalHeading);
            startRightCircle = DubinsMath.GetRightCircleCenterPos(startPos, startHeading);
            startLeftCircle = DubinsMath.GetLeftCircleCenterPos(startPos, startHeading);
        }

        //Calculate the path lengths of all Dubins paths by using tangent points
        private void CalculateDubinsPathsLengths()
        {
            const double eps = 1e-4;
            const double epsSqr = eps * eps;

            //RSR (if start and goal right circles coincide, use exact single-circle arc)
            if ((startRightCircle - goalRightCircle).GetLengthSquared() > epsSqr)
            {
                Get_RSR_Length();
            }
            else
            {
                Get_SingleCircle_Length(startRightCircle, false, PathType.RSR);
            }

            //LSL (if start and goal left circles coincide, use exact single-circle arc)
            if ((startLeftCircle - goalLeftCircle).GetLengthSquared() > epsSqr)
            {
                Get_LSL_Length();
            }
            else
            {
                Get_SingleCircle_Length(startLeftCircle, true, PathType.LSL);
            }

            //RSL and LSR work when circles do not overlap (allow exact tangency within eps)
            double minInnerDist = (2.0 * CDubins.turningRadius) - eps;

            //RSL
            if ((startRightCircle - goalLeftCircle).GetLength() >= minInnerDist)
            {
                Get_RSL_Length();
            }

            //LSR
            if ((startLeftCircle - goalRightCircle).GetLength() >= minInnerDist)
            {
                Get_LSR_Length();
            }

            //With LRL and RLR paths, distance between circles must be <= 4 * r
            //Skip degenerate D ~ 0 where circles coincide (handled by single-circle RSR/LSL)
            double maxCurvedDist = (4.0 * CDubins.turningRadius) + eps;

            //RLR
            double dRLR = (startRightCircle - goalRightCircle).GetLength();
            if (dRLR > eps && dRLR <= maxCurvedDist)
            {
                Get_RLR_Length();
            }

            //LRL
            double dLRL = (startLeftCircle - goalLeftCircle).GetLength();
            if (dLRL > eps && dLRL <= maxCurvedDist)
            {
                Get_LRL_Length();
            }
        }

        private void Get_SingleCircle_Length(vec2 circleCenter, bool isLeft, PathType pathType)
        {
            double length1 = DubinsMath.GetArcLength(circleCenter, startPos, goalPos, isLeft);
            OneDubinsPath pathData = new OneDubinsPath(length1, 0, 0, goalPos, goalPos, pathType)
            {
                segment2Turning = false
            };
            pathData.SetIfTurningRight(!isLeft, false, !isLeft);
            pathDataList.Add(pathData);
        }

        //RSR
        private void Get_RSR_Length()
        {
            DubinsMath.LSLorRSR(startRightCircle, goalRightCircle, false, out vec2 startTangent, out vec2 goalTangent);
            double length1 = DubinsMath.GetArcLength(startRightCircle, startPos, startTangent, false);
            double length2 = (startTangent - goalTangent).GetLength();
            double length3 = DubinsMath.GetArcLength(goalRightCircle, goalTangent, goalPos, false);

            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSR)
            {
                segment2Turning = false
            };
            pathData.SetIfTurningRight(true, false, true);
            pathDataList.Add(pathData);
        }

        //LSL
        private void Get_LSL_Length()
        {
            DubinsMath.LSLorRSR(startLeftCircle, goalLeftCircle, true, out vec2 startTangent, out vec2 goalTangent);
            double length1 = DubinsMath.GetArcLength(startLeftCircle, startPos, startTangent, true);
            double length2 = (startTangent - goalTangent).GetLength();
            double length3 = DubinsMath.GetArcLength(goalLeftCircle, goalTangent, goalPos, true);

            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSL)
            {
                segment2Turning = false
            };
            pathData.SetIfTurningRight(false, false, false);
            pathDataList.Add(pathData);
        }

        //RSL
        private void Get_RSL_Length()
        {
            DubinsMath.RSLorLSR(startRightCircle, goalLeftCircle, false, out vec2 startTangent, out vec2 goalTangent);
            double length1 = DubinsMath.GetArcLength(startRightCircle, startPos, startTangent, false);
            double length2 = (startTangent - goalTangent).GetLength();
            double length3 = DubinsMath.GetArcLength(goalLeftCircle, goalTangent, goalPos, true);

            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSL)
            {
                segment2Turning = false
            };
            pathData.SetIfTurningRight(true, false, false);
            pathDataList.Add(pathData);
        }

        //LSR
        private void Get_LSR_Length()
        {
            DubinsMath.RSLorLSR(startLeftCircle, goalRightCircle, true, out vec2 startTangent, out vec2 goalTangent);
            double length1 = DubinsMath.GetArcLength(startLeftCircle, startPos, startTangent, true);
            double length2 = (startTangent - goalTangent).GetLength();
            double length3 = DubinsMath.GetArcLength(goalRightCircle, goalTangent, goalPos, false);

            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSR)
            {
                segment2Turning = false
            };
            pathData.SetIfTurningRight(false, false, true);
            pathDataList.Add(pathData);
        }

        //RLR
        private void Get_RLR_Length()
        {
            DubinsMath.GetRLRorLRLTangents(
                startRightCircle,
                goalRightCircle,
                false,
                out vec2 startTangent,
                out vec2 goalTangent,
                out vec2 middleCircle);

            double length1 = DubinsMath.GetArcLength(startRightCircle, startPos, startTangent, false);
            double length2 = DubinsMath.GetArcLength(middleCircle, startTangent, goalTangent, true);
            double length3 = DubinsMath.GetArcLength(goalRightCircle, goalTangent, goalPos, false);

            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RLR)
            {
                segment2Turning = true
            };
            pathData.SetIfTurningRight(true, false, true);
            pathDataList.Add(pathData);
        }

        //LRL
        private void Get_LRL_Length()
        {
            DubinsMath.GetRLRorLRLTangents(
                startLeftCircle,
                goalLeftCircle,
                true,
                out vec2 startTangent,
                out vec2 goalTangent,
                out vec2 middleCircle);

            double length1 = DubinsMath.GetArcLength(startLeftCircle, startPos, startTangent, true);
            double length2 = DubinsMath.GetArcLength(middleCircle, startTangent, goalTangent, false);
            double length3 = DubinsMath.GetArcLength(goalLeftCircle, goalTangent, goalPos, true);

            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LRL)
            {
                segment2Turning = true
            };
            pathData.SetIfTurningRight(false, true, false);
            pathDataList.Add(pathData);
        }

        private void GeneratePathCoordinates()
        {
            for (int i = 0; i < pathDataList.Count; i++)
            {
                GetTotalPath(pathDataList[i]);
            }
        }

        private void GetTotalPath(OneDubinsPath pathData)
        {
            List<vec2> finalPath = new List<vec2>();
            vec2 currentPos = startPos;
            double theta = startHeading;

            finalPath.Add(currentPos);

            //First
            if (pathData.length1 > 1e-4)
            {
                int segments = (int)Math.Floor(pathData.length1 / CDubins.driveDistance);
                DubinsMath.AddCoordinatesToPath(
                    ref currentPos,
                    ref theta,
                    finalPath,
                    segments,
                    true,
                    pathData.segment1TurningRight);
            }

            //Second
            if (pathData.length2 > 1e-4)
            {
                int segments = (int)Math.Floor(pathData.length2 / CDubins.driveDistance);
                DubinsMath.AddCoordinatesToPath(
                    ref currentPos,
                    ref theta,
                    finalPath,
                    segments,
                    pathData.segment2Turning,
                    pathData.segment2TurningRight);
            }

            //Third
            if (pathData.length3 > 1e-4)
            {
                int segments = (int)Math.Floor(pathData.length3 / CDubins.driveDistance);
                DubinsMath.AddCoordinatesToPath(
                    ref currentPos,
                    ref theta,
                    finalPath,
                    segments,
                    true,
                    pathData.segment3TurningRight);
            }

            finalPath.Add(new vec2(goalPos.easting, goalPos.northing));
            pathData.pathCoordinates = finalPath;
        }
    }

    public static class DubinsMath
    {
        public static vec2 GetRightCircleCenterPos(vec2 circlePos, double heading)
        {
            vec2 rightCirclePos = new vec2(0, 0)
            {
                easting = circlePos.easting + (CDubins.turningRadius * Math.Sin(heading + glm.PIBy2)),
                northing = circlePos.northing + (CDubins.turningRadius * Math.Cos(heading + glm.PIBy2))
            };
            return rightCirclePos;
        }

        public static vec2 GetLeftCircleCenterPos(vec2 circlePos, double heading)
        {
            vec2 rightCirclePos = new vec2(0, 0)
            {
                easting = circlePos.easting + (CDubins.turningRadius * Math.Sin(heading - glm.PIBy2)),
                northing = circlePos.northing + (CDubins.turningRadius * Math.Cos(heading - glm.PIBy2))
            };
            return rightCirclePos;
        }

        //Outer tangent (LSL and RSR)
        public static void LSLorRSR(vec2 startCircle, vec2 goalCircle, bool isBottom,
                                        out vec2 startTangent, out vec2 goalTangent)
        {
            double theta = glm.PIBy2;
            theta += Math.Atan2(goalCircle.northing - startCircle.northing, goalCircle.easting - startCircle.easting);
            if (isBottom) theta += Math.PI;

            double xT1 = startCircle.easting + (CDubins.turningRadius * Math.Cos(theta));
            double zT1 = startCircle.northing + (CDubins.turningRadius * Math.Sin(theta));

            vec2 dirVec = goalCircle - startCircle;
            double xT2 = xT1 + dirVec.easting;
            double zT2 = zT1 + dirVec.northing;

            startTangent = new vec2(xT1, zT1);
            goalTangent = new vec2(xT2, zT2);
        }

        //Inner tangent (RSL and LSR)
        public static void RSLorLSR(
            vec2 startCircle,
            vec2 goalCircle,
            bool isBottom,
            out vec2 startTangent,
            out vec2 goalTangent)
        {
            double D = (startCircle - goalCircle).GetLength();
            if (D < 1e-9) D = 1e-9;

            double ratio = (2.0 * CDubins.turningRadius) / D;
            if (ratio > 1.0) ratio = 1.0;
            else if (ratio < -1.0) ratio = -1.0;
            double theta = Math.Acos(ratio);

            if (isBottom) theta *= -1.0;
            theta += Math.Atan2(goalCircle.northing - startCircle.northing, goalCircle.easting - startCircle.easting);

            double xT1 = startCircle.easting + (CDubins.turningRadius * Math.Cos(theta));
            double zT1 = startCircle.northing + (CDubins.turningRadius * Math.Sin(theta));

            double xT1_tmp = startCircle.easting + (2.0 * CDubins.turningRadius * Math.Cos(theta));
            double zT1_tmp = startCircle.northing + (2.0 * CDubins.turningRadius * Math.Sin(theta));

            vec2 dirVec = goalCircle - new vec2(xT1_tmp, zT1_tmp);

            double xT2 = xT1 + dirVec.easting;
            double zT2 = zT1 + dirVec.northing;

            startTangent = new vec2(xT1, zT1);
            goalTangent = new vec2(xT2, zT2);
        }

        //Get the RLR or LRL tangent points
        public static void GetRLRorLRLTangents(
            vec2 startCircle,
            vec2 goalCircle,
            bool isLRL,
            out vec2 startTangent,
            out vec2 goalTangent,
            out vec2 middleCircle)
        {
            double D = (startCircle - goalCircle).GetLength();

            double ratio = D / (4.0 * CDubins.turningRadius);
            if (ratio > 1.0) ratio = 1.0;
            else if (ratio < -1.0) ratio = -1.0;
            double theta = Math.Acos(ratio);

            vec2 V1 = goalCircle - startCircle;
            if (isLRL)
                theta = Math.Atan2(V1.northing, V1.easting) + theta;
            else
                theta = Math.Atan2(V1.northing, V1.easting) - theta;

            double x = startCircle.easting + (2.0 * CDubins.turningRadius * Math.Cos(theta));
            double z = startCircle.northing + (2.0 * CDubins.turningRadius * Math.Sin(theta));
            middleCircle = new vec2(x, z);

            vec2 V2 = (startCircle - middleCircle).Normalize();
            vec2 V3 = (goalCircle - middleCircle).Normalize();
            V2 *= CDubins.turningRadius;
            V3 *= CDubins.turningRadius;

            startTangent = middleCircle + V2;
            goalTangent = middleCircle + V3;
        }

        //Calculate arc length safely without 0 -> 2*PI wraparound at coincident points
        public static double GetArcLength(
            vec2 circleCenterPos,
            vec2 startPos,
            vec2 goalPos,
            bool isLeftCircle)
        {
            vec2 V1 = startPos - circleCenterPos;
            vec2 V2 = goalPos - circleCenterPos;

            double theta = Math.Atan2(V2.northing, V2.easting) - Math.Atan2(V1.northing, V1.easting);
            if (isLeftCircle)
            {
                while (theta < 0) theta += 2.0 * Math.PI;
                while (theta >= 2.0 * Math.PI) theta -= 2.0 * Math.PI;
            }
            else
            {
                theta = -theta;
                while (theta < 0) theta += 2.0 * Math.PI;
                while (theta >= 2.0 * Math.PI) theta -= 2.0 * Math.PI;
            }

            if (theta < 1e-6 || (2.0 * Math.PI - theta) < 1e-6) return 0.0;
            return theta * CDubins.turningRadius;
        }

        public static void AddCoordinatesToPath(
            ref vec2 currentPos,
            ref double theta,
            List<vec2> finalPath,
            int segments,
            bool isTurning,
            bool isTurningRight)
        {
            for (int i = 0; i <= segments; i++)
            {
                currentPos.easting += CDubins.driveDistance * Math.Sin(theta);
                currentPos.northing += CDubins.driveDistance * Math.Cos(theta);

                if (isTurning)
                {
                    double turnParameter = isTurningRight ? 1.0 : -1.0;
                    theta += (CDubins.driveDistance / CDubins.turningRadius) * turnParameter;
                }

                finalPath.Add(currentPos);
            }
        }
    }

    public class OneDubinsPath
    {
        public double totalLength;
        public double length1, length2, length3;
        public vec2 tangent1, tangent2;
        public PathType pathType;
        public List<vec2> pathCoordinates;
        public bool segment2Turning;
        public bool segment1TurningRight, segment2TurningRight, segment3TurningRight;

        public OneDubinsPath(double length1, double length2, double length3, vec2 tangent1, vec2 tangent2, PathType pathType)
        {
            this.totalLength = length1 + length2 + length3;
            this.length1 = length1;
            this.length2 = length2;
            this.length3 = length3;
            this.tangent1 = tangent1;
            this.tangent2 = tangent2;
            this.pathType = pathType;
        }

        public void SetIfTurningRight(bool segment1TurningRight, bool segment2TurningRight, bool segment3TurningRight)
        {
            this.segment1TurningRight = segment1TurningRight;
            this.segment2TurningRight = segment2TurningRight;
            this.segment3TurningRight = segment3TurningRight;
        }
    }
}