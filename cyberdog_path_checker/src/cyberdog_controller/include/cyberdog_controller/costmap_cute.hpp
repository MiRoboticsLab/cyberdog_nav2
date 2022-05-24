class CostMapCubes {
 public:
  typedef struct Point {
    int x;
    int y;
  } Point_T;

  typedef struct Cube {
    Point_T point;
    unsigned char value;
    unsigned char nai - value;
  } Cube_T;

  std::vector<Cube_T> points;

 public:
  bool isValidCost(const unsigned char cost) {
    return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
           cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
           cost != nav2_costmap_2d::NO_INFORMATION;
  }

  std::vector<Point_T> getEdgesPoint();
  void fromCostmap(const nav2_costmap_2d::Costmap2DROS &master_grid) {
    unsigned char *master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(),
                 size_y = master_grid.getSizeInCellsY();

    // get index of the array.
    // int index = master_grid.getIndex(mx, my);
    for (int i = 0; i < size_x; i++) {
      for (int j = 0; j < size_y; j++) {
        // int index = master_grid.getIndex(i, j);
        // the index is the costmap buffer index;
        if (isValidCost(master_grid.getCost(i, j))) {
          // the cost is invalid
          // set the nebigerhould field
        }
      }
    }
  }
}

////////////////

#include <vector>
typedef std::vector<int>
    ivChainCode;
// 轮廓跟踪
// 1. pImageData   图像数据
// 2. nWidth       图像宽度     // master_grid.getSizeInCellsX()
// 3. nHeight      图像高度     // master_grid.getSizeInCellsY()
// 4. nWidthStep   图像行大小   // master_grid.getSizeInCellsX()
// 5. pStart       起始点      // given by tracjotry checker.
// 6. pChainCode   链码表      // result
bool TracingContour(unsigned char *pImageData, int nWidth, int nHeight,
                    int nWidthStep, POINT *pStart, ivChainCode *pChainCode) {
  int i = 0;
  int j = 0;
  int k = 0;
  int x = 0;
  int y = 0;
  bool bTracing = false;
  POINT ptCurrent = {0, 0};
  POINT ptTemp = {0, 0};
  unsigned char *pLine = NULL;
  const POINT ptOffset[8] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                             {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
  // 清空起始点与链码表
  pStart->x = 0;
  pStart->y = 0;
  pChainCode->clear();
  // 轮廓起点
  for (y = 0; y < nHeight; y++) {
    pLine = pImageData + nWidthStep * y;
    for (x = 0; x < nWidth; x++) {
      if (pLine[x] == 0xFF) {
        bTracing = true;
        pStart->x = x;
        pStart->y = y;
        ptCurrent.x = x;
        ptCurrent.y = y;
      }
    }
  }
  // 轮廓跟踪
  while (bTracing) {
    bTracing = false;
    for (i = 0; i < 8; i++, k++) {
      k &= 0x07;
      x = ptCurrent.x + ptOffset[k].x;
      y = ptCurrent.y + ptOffset[k].y;
      if (x >= 0 && x < nWidth && y >= 0 && y < nHeight) {
        // 判断是否为轮廓点
        if (pImageData[nWidthStep * y + x] == 0xFF) {
          // isValidCost(master_grid.getCost(x, y))
          for (j = 0; j < 8; j += 2) {
            ptTemp.x = x + ptOffset[j].x;
            ptTemp.y = y + ptOffset[j].y;
            if (ptTemp.x >= 0 && ptTemp.x < nWidth && ptTemp.y >= 0 &&
                ptTemp.y < nHeight) {
              if (pImageData[nWidthStep * ptTemp.y + ptTemp.x] == 0) {
                // // isValidCost(master_grid.getCost(ptTemp.x, ptTemp.y))
                bTracing = true;
                ptCurrent.x = x;
                ptCurrent.y = y;
                pChainCode->push_back(k);
                break;
              }
            }
          }
        }
      }
      if (bTracing) {
        // 如果当前点为轮廓起点
        if (pStart->x == ptCurrent.x && pStart->y == ptCurrent.y) {
          // 则跟踪完毕
          bTracing = false;
        }
        break;
      }
    }
    k += 0x06;
  }
  return true;
}
// 轮廓绘制
// 1. pImageData   图像数据
// 2. nWidth       图像宽度
// 3. nHeight      图像高度
// 4. nWidthStep   图像行大小
// 5. ptStart      起始点
// 6. ChainCode    链码表
bool DrawContour(unsigned char *pImageData, int nWidth, int nHeight,
                 int nWidthStep, POINT ptStart, ivChainCode ChainCode) {
  POINT ptCurrent = {0, 0};
  const POINT ptOffset[8] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                             {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
  // 清空图像
  memset(pImageData, 0, nWidthStep * nHeight);
  // 轮廓绘制
  ptCurrent.x = ptStart.x;
  ptCurrent.y = ptStart.y;
  pImageData[nWidthStep * ptCurrent.y + ptCurrent.x] = 0xFF;
  for (ivChainCode::iterator i = ChainCode.begin(); i != ChainCode.end(); i++) {
    ptCurrent.x += ptOffset[*i].x;
    ptCurrent.y += ptOffset[*i].y;
    pImageData[nWidthStep * ptCurrent.y + ptCurrent.x] = 0xFF;
  }
  return true;
}

////////////////////

void BoundaryTracking() {
  // 指向源图像的指针
  LPBYTE lpSrc;
  LPBYTE p_data;
  // 指向缓存图像的指针
  LPBYTE lpDst;
  // 指向缓存DIB图像的指针
  LPBYTE temp;
  long wide;
  long height;
  //循环变量
  long i;
  long j;
  //像素值
  long pixel;
  //是否找到起始点及回到起始点
  bool bFindStartPoint;
  //是否扫描到一个边界点
  bool bFindPoint;
  //起始边界点与当前边界点
  Point StartPoint, CurrentPoint;
  //八个方向和起始扫描方向
  int Direction[8][2] = {{-1, 1}, {0, 1},  {1, 1},   {1, 0},
                         {1, -1}, {0, -1}, {-1, -1}, {-1, 0}};
  int BeginDirect;
  p_data = GetData();
  wide = GetWidth();
  height = GetHeight();
  for (j = 0; j < height; j++) {
    for (i = 0; i < wide; i++) {
      lpSrc = (LPBYTE)p_data + wide * j + i;
      if (*lpSrc > 127)
        *lpSrc = 255;
      else
        *lpSrc = 0;
    }
  }

  // 暂时分配内存，以保存新图像
  temp = new BYTE[wide * height];
  // 初始化新分配的内存，设定初始值为255
  lpDst = temp;
  memset(lpDst, (BYTE)255, wide * height);
  //先找到最左上方的边界点
  bFindStartPoint = false;
  for (j = 0; j < height && !bFindStartPoint; j++) {
    for (i = 0; i < wide && !bFindStartPoint; i++) {
      // 指向源图像倒数第j行，第i个象素的指针
      lpSrc = (LPBYTE)(p_data + wide * j + i);
      //取得当前指针处的像素值，注意要转换为unsigned char型
      pixel = *lpSrc;
      if (pixel == 0) {
        bFindStartPoint = true;
        StartPoint.Height = j;
        StartPoint.Width = i;
        // 指向目标图像倒数第j行，第i个象素的指针
        lpDst = (LPBYTE)(temp + wide * j + i);
        *lpDst = 0;
      }
    }
  }

  //由于起始点是在左下方，故起始扫描沿左上方向
  BeginDirect = 0;
  //跟踪边界
  bFindStartPoint = false;
  //从初始点开始扫描
  CurrentPoint.Height = StartPoint.Height;
  CurrentPoint.Width = StartPoint.Width;
  while (!bFindStartPoint) {
    bFindPoint = false;
    while (!bFindPoint) {
      //沿扫描方向查看一个像素
      lpSrc =
          (LPBYTE)(p_data +
                   wide * (CurrentPoint.Height + Direction[BeginDirect][1]) +
                   (CurrentPoint.Width + Direction[BeginDirect][0]));
      pixel = *lpSrc;
      if (pixel == 0) {
        bFindPoint = true;
        CurrentPoint.Height = CurrentPoint.Height + Direction[BeginDirect][1];
        CurrentPoint.Width = CurrentPoint.Width + Direction[BeginDirect][0];
        if (CurrentPoint.Height == StartPoint.Height &&
            CurrentPoint.Width == StartPoint.Width) {
          bFindStartPoint = true;
        }
        lpDst =
            (LPBYTE)(temp + wide * CurrentPoint.Height + CurrentPoint.Width);
        *lpDst = 0;
        //扫描的方向逆时针旋转两格
        BeginDirect--;
        if (BeginDirect == -1) BeginDirect = 7;
        BeginDirect--;
        if (BeginDirect == -1) BeginDirect = 7;
      } else {
        //扫描方向顺时针旋转一格
        BeginDirect++;
        if (BeginDirect == 8) BeginDirect = 0;
      }
    }
  }
  // 复制图像
  memcpy(p_data, temp, wide * height);
  // 释放内存
  delete temp;
}