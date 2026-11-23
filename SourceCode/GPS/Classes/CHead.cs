using System;

namespace AgOpenGPS
{
    public partial class CBoundary
    {
        public bool isHeadlandOn;

        public bool isToolInHeadland,
            isToolOuterPointsInHeadland, isSectionControlledByHeadland,
            isHeadlandTurnLocked;

        public vec2? HeadlandNearestPoint { get; private set; } = null;
        public double? HeadlandDistance { get; private set; } = null;

        public bool IsHeadlandTurnLocked()
        {
            if (!isHeadlandOn || bndList.Count == 0 || bndList[0].hdLine.Count == 0)
                return false;

            if (mf.ct.isContourBtnOn || mf.trk.gArr == null || mf.trk.idx < 0 || mf.trk.idx >= mf.trk.gArr.Count)
                return false;

            bool inOrTouchingHeadland =
                !IsPointInsideHeadArea(mf.pivotAxlePos.ToVec2())
                || !IsPointInsideHeadArea(mf.toolPivotPos.ToVec2())
                || mf.tool.isLeftSideInHeadland
                || mf.tool.isRightSideInHeadland
                || mf.yt.isYouTurnTriggered;

            if (!inOrTouchingHeadland)
                return false;

            double targetHeading;
            if (mf.trk.gArr[mf.trk.idx].mode == TrackMode.AB)
            {
                targetHeading = mf.ABLine.abHeading;
            }
            else
            {
                if (mf.curve.curList == null || mf.curve.curList.Count == 0)
                    return false;

                double minDistSq = double.MaxValue;
                int bestIdx = -1;
                for (int i = 0; i < mf.curve.curList.Count; i++)
                {
                    double dSq = glm.DistanceSquared(mf.pivotAxlePos, mf.curve.curList[i]);
                    if (dSq < minDistSq)
                    {
                        minDistSq = dSq;
                        bestIdx = i;
                    }
                }
                if (bestIdx < 0) return false;
                targetHeading = mf.curve.curList[bestIdx].heading;
            }

            double diff = Math.Abs(mf.pivotAxlePos.heading - targetHeading) % Math.PI;
            if (diff > glm.PIBy2) diff = Math.PI - diff;

            return diff > glm.toRadians(10.0);
        }

        public void SetHydPosition()
        {
            if (mf.vehicle.isHydLiftOn && mf.avgSpeed > 0.2 && !mf.isReverse)
            {
                if (isToolInHeadland)
                {
                    mf.p_239.pgn[mf.p_239.hydLift] = 2;
                    if (mf.sounds.isHydLiftChange != isToolInHeadland)
                    {
                        if (mf.sounds.isHydLiftSoundOn) mf.sounds.sndHydLiftUp.Play();
                        mf.sounds.isHydLiftChange = isToolInHeadland;
                    }
                }
                else
                {
                    mf.p_239.pgn[mf.p_239.hydLift] = 1;
                    if (mf.sounds.isHydLiftChange != isToolInHeadland)
                    {
                        if (mf.sounds.isHydLiftSoundOn) mf.sounds.sndHydLiftDn.Play();
                        mf.sounds.isHydLiftChange = isToolInHeadland;
                    }
                }
            }
        }

        public void WhereAreToolCorners()
        {
            if (bndList.Count > 0 && bndList[0].hdLine.Count > 0)
            {
                bool isLeftInWk, isRightInWk = true;

                for (int j = 0; j < mf.tool.numOfSections; j++)
                {
                    isLeftInWk = j == 0 ? IsPointInsideHeadArea(mf.section[j].leftPoint) : isRightInWk;
                    isRightInWk = IsPointInsideHeadArea(mf.section[j].rightPoint);

                    //save left side
                    if (j == 0)
                        mf.tool.isLeftSideInHeadland = !isLeftInWk;

                    //merge the two sides into in or out
                    mf.section[j].isInHeadlandArea = !isLeftInWk && !isRightInWk;
                }

                //save right side
                mf.tool.isRightSideInHeadland = !isRightInWk;

                //is the tool in or out based on endpoints
                isToolOuterPointsInHeadland = mf.tool.isLeftSideInHeadland && mf.tool.isRightSideInHeadland;

                //10 derece yastikbasi donus kilidi
                isHeadlandTurnLocked = IsHeadlandTurnLocked();
            }
            else
            {
                isHeadlandTurnLocked = false;
            }
        }

        public void WhereAreToolLookOnPoints()
        {
            if (bndList.Count > 0 && bndList[0].hdLine.Count > 0)
            {
                bool isLookRightIn = false;

                vec3 toolFix = mf.toolPivotPos;
                double sinAB = Math.Sin(toolFix.heading);
                double cosAB = Math.Cos(toolFix.heading);

                //generated box for finding closest point
                double pos = 0;
                double mOn = (mf.tool.lookAheadDistanceOnPixelsRight - mf.tool.lookAheadDistanceOnPixelsLeft) / mf.tool.rpWidth;

                for (int j = 0; j < mf.tool.numOfSections; j++)
                {
                    bool isLookLeftIn = j == 0 ? IsPointInsideHeadArea(new vec2(
                        mf.section[j].leftPoint.easting + (sinAB * mf.tool.lookAheadDistanceOnPixelsLeft * 0.1),
                        mf.section[j].leftPoint.northing + (cosAB * mf.tool.lookAheadDistanceOnPixelsLeft * 0.1))) : isLookRightIn;

                    pos += mf.section[j].rpSectionWidth;
                    double endHeight = (mf.tool.lookAheadDistanceOnPixelsLeft + (mOn * pos)) * 0.1;

                    isLookRightIn = IsPointInsideHeadArea(new vec2(
                        mf.section[j].rightPoint.easting + (sinAB * endHeight),
                        mf.section[j].rightPoint.northing + (cosAB * endHeight)));

                    mf.section[j].isLookOnInHeadland = !isLookLeftIn && !isLookRightIn;
                }
            }
        }

        public bool IsPointInsideHeadArea(vec2 pt)
        {
            //if inside outer boundary, then potentially add
            if (bndList[0].hdLine.IsPointInPolygon(pt))
            {
                for (int i = 1; i < bndList.Count; i++)
                {
                    if (bndList[i].hdLine.IsPointInPolygon(pt))
                    {
                        return false;
                    }
                }
                return true;
            }
            return false;
        }
        public void CheckHeadlandProximity()
        {
            if (!isHeadlandOn || bndList.Count == 0 || bndList[0].hdLine.Count < 2)
            {
                HeadlandNearestPoint = null;
                HeadlandDistance = null;
                return;
            }

            vec3 vehiclePos = mf.toolPivotPos;

            vec2? nearest = glm.RaycastToPolygon(vehiclePos, bndList[0].hdLine);
            if (!nearest.HasValue)
            {
                HeadlandNearestPoint = null;
                HeadlandDistance = null;
                return;
            }

            vec2 nearestVal = nearest.Value;
            double distance = glm.Distance(vehiclePos.ToVec2(), nearestVal);

            HeadlandNearestPoint = nearestVal;
            HeadlandDistance = distance;

            // Yastikbasi yaklasma sesi devre disi (U-donus alarmi ile isBoundAlarming cakismasini onlemek icin)
        }

    }
}
