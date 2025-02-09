// #pragma region ZigZag
// std::vector<std::array<float,2>> zigZagPoints;
// #pragma endregion

// void findClosestCorner(std::array<std::array<float, 2>, 4> cornerCoords, float posX, float posY, float &closestX, float &closestY, int &closestInd){
//     std::array<float,2> closestCoordinate;
//     float closestDistance;
    
//     closestInd = 0;
//     closestCoordinate = cornerCoords[0];
//     closestDistance = distanceBetween(posX, posY, closestCoordinate[0], closestCoordinate[1]);
    
//     float distance;
//     for(int i = 1; i < 4; i++){
//         distance = distanceBetween(posX, posY, cornerCoords[i][0], cornerCoords[i][1]);
//         if(distance < closestDistance){
//             closestCoordinate = cornerCoords[i];
//             closestDistance = distance;
//             closestInd = i;
//         }
//     }
    
//     closestX = closestCoordinate[0];
//     closestY = closestCoordinate[1];
// }

// void computeZigZag(std::array<std::array<float,2>,4> cornerCoords, int closestInd, std::vector<std::array<float,2>> &zigZagPoints){
//     // Key Parameters
//     float minSegmentLength = 1.5;
    
//     // 1. Get the indices of the adjacent corners to the closest corner
//     int nextInd = closestInd + 1;
//     wrapIntegerIndexAroundRange(nextInd, 0, 3);
    
//     int prevInd = closestInd - 1;
//     wrapIntegerIndexAroundRange(prevInd, 0, 3);
    
//     // 2. Find which of the two adjacent corners is farther away
//     int indAStart = closestInd;
//     int indAEnd;
//     float distanceNext = distanceBetween(cornerCoords[closestInd][0], cornerCoords[closestInd][1], cornerCoords[nextInd][0], cornerCoords[nextInd][1]);
//     float distancePrev = distanceBetween(cornerCoords[closestInd][0], cornerCoords[closestInd][1], cornerCoords[prevInd][0], cornerCoords[prevInd][1]);
    
//     if(distanceNext > distancePrev){
//         indAEnd = nextInd;
//     }
//     else{
//         indAEnd = prevInd;
//     }
    
//     // 3. Find A-side and B-side displacements; A-side is the closer long span and B-side is the farther long span
//     float dxA;
//     float dyA;
//     float dA;
    
//     dxA = cornerCoords[indAEnd][0]-cornerCoords[indAStart][0];
//     dyA = cornerCoords[indAEnd][1]-cornerCoords[indAStart][1];
//     dA = sqrt(pow(dxA, 2) + pow(dyA, 2));
    
//     float dxB;
//     float dyB;
    
//     int indBStart = indAStart + (indAStart-indAEnd);
//     int indBEnd = indAStart + 2*(indAStart-indAEnd);
    
//     wrapIntegerIndexAroundRange(indBStart, 0, 3);
//     wrapIntegerIndexAroundRange(indBEnd, 0, 3);
    
//     dxB = cornerCoords[indBEnd][0]-cornerCoords[indBStart][0];
//     dyB = cornerCoords[indBEnd][1]-cornerCoords[indBStart][1];

//     //4. Calculate nSegments and segmentDisplacementA/segmentDisplacementB (sxA, syA, sxB, syB)
//     int nSegmentsA = std::ceil(dA/minSegmentLength);
    
//     int nSegmentsB = nSegmentsA + 1;
    
//     float sxA = dxA / nSegmentsA;
//     float syA = dyA / nSegmentsA;
    
//     float sxB = dxB / nSegmentsB;
//     float syB = dyB / nSegmentsB;
    
//     //5. Calculate zigzag positions
//     float pxA;
//     float pyA;
//     float pxB;
//     float pyB;
    
//     for(int i = 0; i < nSegmentsA + 1; i++){
//         pxA = cornerCoords[indAStart][0] + sxA*i;
//         pyA = cornerCoords[indAStart][1] + syA*i;
//         zigZagPoints.push_back({pxA, pyA});
        
//         if( i < nSegmentsB - 1){
//             pxB = cornerCoords[indBStart][0] + sxB*(i+1);
//             pyB = cornerCoords[indBStart][1] + syB*(i+1);
//             zigZagPoints.push_back({pxB, pyB});
//         }
//     }
    
//     // for(int i = 0; i < zigZagPoints.size(); i++){
//     //     std::cout << "(" << zigZagPoints[i][0]<< "," << zigZagPoints[i][1] << ")" << std::endl;
//     // } 
//}





    // std::array<std::array<float, 2>, 4> cornerCoords = {{
    //     {-1.929,1.346},
    //     {-1.7069999999999999,-1.0},
    //     {2.2,-1.38},
    //     {2.3149999999999995,1.708}
    // }};

    // int closestInd;
    // float closestX;
    // float closestY;

    // findClosestCorner(cornerCoords, posX, posY, closestX, closestY, closestInd);
    // computeZigZag(cornerCoords, closestInd, zigZagPoints);

    // float zpX;
    // float zpY;
    // for(int i = 0; i < zigZagPoints.size(); i++){
    //     zpX = zigZagPoints[i][0];
    //     zpY = zigZagPoints[i][1];

    //     ROS_INFO("zz navigating to: x/y %.2f/%.2f", zpX, zpY);

    //     navigateToPositionSmart(zpX, zpY, vel, vel_pub);
    // }