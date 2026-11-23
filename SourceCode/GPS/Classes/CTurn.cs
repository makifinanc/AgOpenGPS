using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace AgOpenGPS
{
    public partial class CBoundary
    {

        public int turnSelected;
        public double iE = 0, iN = 0;
        // the list of possible bounds points
        //public List<vec3> turnClosestList = new List<vec3>();

        //point at the farthest turn segment from pivotAxle
        //public vec3 closestTurnPt = new vec3(-10000, -10000, 9);

        public int IsPointInsideTurnArea(vec3 pt)
        {
            if (bndList.Count > 0 && bndList[0].turnLine.IsPointInPolygon(pt))
            {
                for (int i = 1; i < bndList.Count; i++)
                {
                    if (bndList[i].isDriveThru) continue;
                    if (bndList[i].turnLine.IsPointInPolygon(pt))
                    {
                        return i;
                    }
                }
                return 0;
            }
            return -1; //is outside border turn
        }

        //public vec3 closePt;

        //public void FoindClosestTurnPoint(vec3 fromPt)
        //{
        //    double eP = fromPt.easting;
        //    double nP = fromPt.northing;
        //    double eAB, nAB;

        //    if (mf.ABLine.isHeadingSameWay)
        //    {
        //        eAB = mf.ABLine.currentLinePtB.easting;
        //        nAB = mf.ABLine.currentLinePtB.northing;
        //    }
        //    else
        //    {
        //        eAB = mf.ABLine.currentLinePtA.easting;
        //        nAB = mf.ABLine.currentLinePtA.northing;
        //    }

        //    turnClosestList.Clear();

        //    for (int j = 0; j < bndList.Count; j++)
        //    {
        //        for (int i = 0; i < mf.bnd.bndList[j].turnLine.Count - 1; i++)
        //        {
        //            int res = mf.yt.GetLineIntersection(
        //                mf.bnd.bndList[j].turnLine[i].easting,
        //                mf.bnd.bndList[j].turnLine[i].northing,
        //                mf.bnd.bndList[j].turnLine[i + 1].easting,
        //                mf.bnd.bndList[j].turnLine[i + 1].northing,
        //                eP, nP, eAB, nAB, ref iE, ref iN
        //                );

        //            if (res == 1)
        //            {
        //                closePt.easting = iE;
        //                closePt.northing = iN;

        //                double hed = Math.Atan2(mf.bnd.bndList[j].turnLine[i + 1].easting - mf.bnd.bndList[j].turnLine[i].easting,
        //                    mf.bnd.bndList[j].turnLine[i + 1].northing - mf.bnd.bndList[j].turnLine[i].northing);
        //                if (hed < 0) hed += glm.twoPI;
        //                closePt.heading = hed;
        //                turnClosestList.Add(closePt);
        //            }
        //        }
        //    }

        //    //determine closest point
        //    double minDistance = double.MaxValue;

        //    if (turnClosestList.Count == 1)
        //    {
        //        closestTurnPt.easting = turnClosestList[0].easting;
        //        closestTurnPt.northing = turnClosestList[0].northing;
        //        closestTurnPt.heading = turnClosestList[0].heading;

        //        if (closestTurnPt.heading < 0) closestTurnPt.heading += glm.twoPI;
        //        return;
        //    }

        //    if (turnClosestList.Count > 0 || turnClosestList.Count != 0)
        //    {
        //        for (int i = 0; i < turnClosestList.Count; i++)
        //        {
        //            double dist = (((fromPt.easting - turnClosestList[i].easting) * (fromPt.easting - turnClosestList[i].easting))
        //                            + ((fromPt.northing - turnClosestList[i].northing) * (fromPt.northing - turnClosestList[i].northing)));

        //            if (minDistance >= dist)
        //            {
        //                minDistance = dist;
        //                closestTurnPt.easting = turnClosestList[i].easting;
        //                closestTurnPt.northing = turnClosestList[i].northing;
        //                closestTurnPt.heading = turnClosestList[i].heading;
        //            }
        //        }
        //        if (closestTurnPt.heading < 0) closestTurnPt.heading += glm.twoPI;
        //    }
        //}

        // Set to false once you are happy; true writes turnline_debug.csv next to AgOpenGPS.exe
        // every time the turn lines are rebuilt (fence points + turn line points).
        private static readonly bool dumpTurnLineDebug = false;

        // one sample of the raw (dense) offset curve + the direction of the fence segment it came from
        private struct OffPt
        {
            public double e, n, tx, ty;
        }

        public void BuildTurnLines()
        {
            if (bndList.Count == 0)
            {
                //mf.TimedMessageBox(1500, " No Boundaries", "No Turn Lines Made");
                return;
            }

            //update the GUI values for boundaries
            mf.fd.UpdateFieldBoundaryGUIAreas();

            //determine how wide a headland space
            double totalHeadWidth = mf.yt.uturnDistanceFromBoundary;

            //inside boundaries
            for (int j = 0; j < bndList.Count; j++)
            {
                bndList[j].turnLine.Clear();
                if (bndList[j].isDriveThru) continue;
                if (bndList[j].fenceLine.Count < 3) continue;

                //1. exact offset of the fence line: dense samples along every offset segment,
                //   invalid ones removed, corners rebuilt from the two neighbouring offset lines
                List<vec3> pts = OffsetFenceLine(j, totalHeadWidth);
                if (pts.Count < 3) continue;

                //2. headings = direction of travel along the line (original CalculateTurnHeadings)
                bndList[j].turnLine.AddRange(pts);
                bndList[j].CalculateTurnHeadings();

                //countExit the reference list of original curve
                int cnt = bndList[j].turnLine.Count;

                //the temp array
                vec3[] arr = new vec3[cnt];

                for (int s = 0; s < cnt; s++)
                {
                    arr[s] = bndList[j].turnLine[s];
                }

                //3. original thinning by accumulated heading change (with the +-2PI wrap fixed)
                double delta = 0;
                bndList[j].turnLine.Clear();

                for (int i = 0; i < arr.Length; i++)
                {
                    if (i == 0)
                    {
                        bndList[j].turnLine.Add(arr[i]);
                        continue;
                    }

                    double step = arr[i - 1].heading - arr[i].heading;
                    if (step > Math.PI) step -= glm.twoPI;
                    else if (step < -Math.PI) step += glm.twoPI;

                    delta += step;
                    if (Math.Abs(delta) > 0.005)
                    {
                        bndList[j].turnLine.Add(arr[i]);
                        delta = 0;
                    }
                }

                if (bndList[j].turnLine.Count > 0)
                {
                    vec3 end = new vec3(bndList[j].turnLine[0].easting,
                        bndList[j].turnLine[0].northing, bndList[j].turnLine[0].heading);
                    bndList[j].turnLine.Add(end);
                }
            }

            if (dumpTurnLineDebug) DumpTurnLineDebug(totalHeadWidth);
        }

        // Offsets fence line b to its LEFT side by d (same side as the original code:
        // inside for the outer boundary, outside for inner boundaries).
        // Returned points are in turn line order (reverse of fence order), headings not set yet.
        private List<vec3> OffsetFenceLine(int b, double d)
        {
            const double step = 1.0;      // sampling distance along every offset segment (m)
            const double tol = 0.03;      // a sample may be this much closer than d to the fence (m)
            const double maxJoin = 3.0;   // longest extension used to rebuild a corner (m)

            List<vec3> fence = bndList[b].fenceLine;
            bool isOuter = (b == 0);
            int cnt = fence.Count;

            //dense raw offset curve
            List<OffPt> raw = new List<OffPt>(cnt * 3);

            for (int i = 0; i < cnt; i++)
            {
                vec3 pa = fence[i];
                vec3 pb = fence[(i + 1) % cnt];

                double dx = pb.easting - pa.easting;
                double dy = pb.northing - pa.northing;
                double len = Math.Sqrt((dx * dx) + (dy * dy));
                if (len < 1e-6) continue;

                double ux = dx / len;     // unit direction of the fence segment
                double uy = dy / len;
                double nx = -uy;          // left normal
                double ny = ux;

                int k = Math.Max(1, (int)Math.Ceiling(len / step));
                for (int m = 0; m < k; m++)
                {
                    double f = (double)m / k;
                    OffPt op = new OffPt();
                    op.e = pa.easting + (dx * f) + (nx * d);
                    op.n = pa.northing + (dy * f) + (ny * d);
                    op.tx = ux;
                    op.ty = uy;
                    raw.Add(op);
                }

                //right turn at pb: the offset lines separate, fill the gap with an arc of radius d
                //next fence point that is really different from pb (the ring may repeat its first point)
                double ex = 0, ey = 0, elen = 0;
                for (int q2 = 2; q2 <= cnt; q2++)
                {
                    vec3 pc = fence[(i + q2) % cnt];
                    ex = pc.easting - pb.easting;
                    ey = pc.northing - pb.northing;
                    elen = Math.Sqrt((ex * ex) + (ey * ey));
                    if (elen > 1e-6) break;
                }
                if (elen < 1e-6) continue;
                ex /= elen;
                ey /= elen;

                double theta = Math.Atan2((ux * ey) - (uy * ex), (ux * ex) + (uy * ey));
                if (theta < -0.05)
                {
                    double a1 = Math.Atan2(ny, nx);
                    int steps = (int)Math.Ceiling(-theta / 0.1);
                    for (int s = 0; s < steps; s++)
                    {
                        double phi = theta * s / steps;
                        double cp = Math.Cos(phi);
                        double sp = Math.Sin(phi);
                        OffPt op = new OffPt();
                        op.e = pb.easting + (Math.Cos(a1 + phi) * d);
                        op.n = pb.northing + (Math.Sin(a1 + phi) * d);
                        op.tx = (ux * cp) - (uy * sp);
                        op.ty = (ux * sp) + (uy * cp);
                        raw.Add(op);
                    }
                }
            }

            //keep only samples that are on the right side and not closer than d to any fence segment
            int total = raw.Count;
            List<int> keep = new List<int>(total);
            for (int i = 0; i < total; i++)
            {
                if (!FarEnoughFromFence(fence, raw[i].e, raw[i].n, d - tol)) continue;
                if (isOuter != bndList[b].fenceLineEar.IsPointInPolygon(new vec3(raw[i].e, raw[i].n, 0))) continue;
                keep.Add(i);
            }

            List<vec3> res = new List<vec3>();
            if (keep.Count < 3) return res;

            for (int q = 0; q < keep.Count; q++)
            {
                int i1 = keep[q];
                int i2 = keep[(q + 1) % keep.Count];
                OffPt p1 = raw[i1];
                res.Add(new vec3(p1.e, p1.n, 0));

                //samples were removed between p1 and p2: that is a corner. Rebuild its exact tip as the
                //crossing of the offset line ending at p1 and the offset line starting at p2.
                int gap = (((i2 - i1) % total) + total) % total;
                if (gap > 1)
                {
                    OffPt p2 = raw[i2];
                    double den = (p1.tx * p2.ty) - (p1.ty * p2.tx);
                    if (Math.Abs(den) > 0.01)
                    {
                        double wx = p2.e - p1.e;
                        double wy = p2.n - p1.n;
                        double t = ((wx * p2.ty) - (wy * p2.tx)) / den;
                        double s = ((wx * p1.ty) - (wy * p1.tx)) / den;

                        if (t >= -0.5 && t <= maxJoin && s >= -maxJoin && s <= 0.5)
                        {
                            double mx = p1.e + (p1.tx * t);
                            double my = p1.n + (p1.ty * t);

                            if (FarEnoughFromFence(fence, mx, my, d - tol)
                                && isOuter == bndList[b].fenceLineEar.IsPointInPolygon(new vec3(mx, my, 0)))
                            {
                                res.Add(new vec3(mx, my, 0));
                            }
                        }
                    }
                }
            }

            res.Reverse();
            return res;
        }

        // true when (px,py) is at least minDist away from every fence SEGMENT (not only from the fence points)
        private static bool FarEnoughFromFence(List<vec3> fence, double px, double py, double minDist)
        {
            if (minDist <= 0) return true;
            double min2 = minDist * minDist;
            int cnt = fence.Count;

            for (int i = 0; i < cnt; i++)
            {
                vec3 a = fence[i];
                vec3 b = fence[(i + 1) % cnt];

                //quick reject with the bounding box of the segment
                if ((px < a.easting - minDist && px < b.easting - minDist)
                    || (px > a.easting + minDist && px > b.easting + minDist)
                    || (py < a.northing - minDist && py < b.northing - minDist)
                    || (py > a.northing + minDist && py > b.northing + minDist)) continue;

                double dx = b.easting - a.easting;
                double dy = b.northing - a.northing;
                double l2 = (dx * dx) + (dy * dy);
                double t = l2 > 0 ? (((px - a.easting) * dx) + ((py - a.northing) * dy)) / l2 : 0;
                if (t < 0) t = 0;
                else if (t > 1) t = 1;

                double ex = a.easting + (t * dx) - px;
                double ey = a.northing + (t * dy) - py;
                if ((ex * ex) + (ey * ey) < min2) return false;
            }
            return true;
        }

        // Temporary diagnostics: writes turnline_debug.csv next to AgOpenGPS.exe
        //   line 1:  D,<uturn distance>
        //   F,<e>,<n>          fence points of every boundary (index in first column after F)
        //   T,<e>,<n>,<head>   turn line points
        private void DumpTurnLineDebug(double d)
        {
            try
            {
                CultureInfo ci = CultureInfo.InvariantCulture;
                string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "turnline_debug.csv");
                List<string> lines = new List<string>();
                lines.Add("D," + d.ToString("F3", ci));

                for (int j = 0; j < bndList.Count; j++)
                {
                    for (int i = 0; i < bndList[j].fenceLine.Count; i++)
                    {
                        lines.Add("F" + j + "," + bndList[j].fenceLine[i].easting.ToString("F3", ci) + ","
                            + bndList[j].fenceLine[i].northing.ToString("F3", ci));
                    }
                    for (int i = 0; i < bndList[j].turnLine.Count; i++)
                    {
                        lines.Add("T" + j + "," + bndList[j].turnLine[i].easting.ToString("F3", ci) + ","
                            + bndList[j].turnLine[i].northing.ToString("F3", ci) + ","
                            + bndList[j].turnLine[i].heading.ToString("F4", ci));
                    }
                }
                File.WriteAllLines(path, lines);
            }
            catch (Exception)
            {
                //diagnostics must never break the field load
            }
        }
    }
}
