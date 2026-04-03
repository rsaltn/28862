////
////public static class Paths {
////    public PathChain startToShoot;
////    public PathChain shootTocollect1;
////    public PathChain collect1;
////    public PathChain return1;
////    public PathChain shootToGate;
////    public PathChain gate;
////    public PathChain gateToCollect;
////    public PathChain return2;
////    public PathChain collect2;
////    public PathChain return3;
////    public PathChain shootTocollect2;
////    public PathChain collect3;
////    public PathChain return4;
////
////    public Paths(Follower follower) {
//        startToShoot = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(29.000, 134.000),
//
//                                new Pose(50.000, 84.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
//
//                .build();
//
//        shootTocollect1 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(50.000, 84.000),
//
//                                new Pose(50.000, 60.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        collect1 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(50.000, 60.000),
//
//                                new Pose(9.000, 60.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        return1 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(9.000, 60.000),
//                                new Pose(39.000, 57.000),
//                                new Pose(50.000, 84.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        shootToGate = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(50.000, 84.000),
//
//                                new Pose(22.000, 65.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        gate = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(22.000, 65.000),
//
//                                new Pose(16.000, 65.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        gateToCollect = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(16.000, 65.000),
//                                new Pose(19.000, 57.000),
//                                new Pose(9.000, 54.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
//
//                .build();
//
//        return2 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(9.000, 54.000),
//
//                                new Pose(50.000, 84.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
//
//                .build();
//
//        collect2 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(50.000, 84.000),
//
//                                new Pose(16.000, 84.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        return3 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(16.000, 84.000),
//
//                                new Pose(50.000, 84.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        shootTocollect2 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(50.000, 84.000),
//
//                                new Pose(50.000, 36.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        collect3 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(50.000, 36.000),
//
//                                new Pose(9.000, 36.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//
//        return4 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(9.000, 36.000),
//
//                                new Pose(50.000, 84.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//
//                .build();
//    }
////}
////