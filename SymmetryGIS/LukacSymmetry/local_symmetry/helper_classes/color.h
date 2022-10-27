#ifndef COLOR_H
#define COLOR_H

#include "LAS/Data/nVector.h"


namespace Symmetry {
    namespace Color {
        static const LAS::Data::Vector4d transparent =            LAS::Data::Vector4d(0.0, 0.0, 0.0, 0.0);
        static const LAS::Data::Vector4d black =                  LAS::Data::Vector4d(0.0, 0.0, 0.0, 1.0);
        static const LAS::Data::Vector4d white =                  LAS::Data::Vector4d(1.0, 1.0, 1.0, 0.0);
        static const LAS::Data::Vector4d gray =                   LAS::Data::Vector4d(0.4, 0.4, 0.4, 1.0);
        static const LAS::Data::Vector4d red =                    LAS::Data::Vector4d(1.0, 0.0, 0.0, 1.0);
        static const LAS::Data::Vector4d blue =                   LAS::Data::Vector4d(0.0, 0.0, 1.0, 1.0);
        static const LAS::Data::Vector4d purple =                 LAS::Data::Vector4d(0.6, 0.2, 0.9, 1.0);
        static const LAS::Data::Vector4d magenta =                LAS::Data::Vector4d(1.0, 0.0, 1.0, 1.0);
        static const LAS::Data::Vector4d voxelLight =             LAS::Data::Vector4d(0.3, 0.3, 0.3, 0.4);
        static const LAS::Data::Vector4d voxelDark =              LAS::Data::Vector4d(0.2, 0.2, 0.2, 0.4);
        static const LAS::Data::Vector4d interestingVoxelLight =  LAS::Data::Vector4d(1.0, 0.8, 0.0, 0.4);
        static const LAS::Data::Vector4d interestingVoxelDark =   LAS::Data::Vector4d(1.0, 0.6, 0.0, 0.4);
        static const LAS::Data::Vector4d symmetryVoxelLightBlue = LAS::Data::Vector4d(0.0, 0.0, 1.0, 0.4);
        static const LAS::Data::Vector4d symmetryVoxelDarkBlue =  LAS::Data::Vector4d(0.0, 0.0, 0.8, 0.4);
        static const LAS::Data::Vector4d symmetryVoxelLightRed =  LAS::Data::Vector4d(1.0, 0.0, 0.0, 0.4);
        static const LAS::Data::Vector4d symmetryVoxelDarkRed =   LAS::Data::Vector4d(0.8, 0.0, 0.0, 0.4);
        static const LAS::Data::Vector4d lineSegment =            LAS::Data::Vector4d(0.0, 0.5, 0.5, 1.0);
        static const LAS::Data::Vector4d symmetryAxis =           LAS::Data::Vector4d(0.0, 0.8, 0.5, 1.0);
        static const LAS::Data::Vector4d lightGreen =             LAS::Data::Vector4d(0.2, 0.9, 0.4, 1.0);
    };
};

#endif // COLOR_H
