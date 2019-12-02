Raspberry Piとモータドライバを用いて自動運転を行う.
目標: 赤の×の前まで移動して止まる（ただし今回は距離5mで固定）

quintic_polynomial_planner.pyは,
PythonRobotics/PathPlanning/QuinticPolynomialsPlannerを用いた.
引用先 : https://github.com/AtsushiSakai/PythonRobotics

go.pyは
vision関数 : カメラで画像取得し,フレーム内の×の重心を算出.
            重心を算出できたかどうか,重心X,重心Y,フレームのWidth, フレームのHeightを返す

calculate_y関数 : 引数に重心X,フレームのWidthをとる.
                  カメラの焦点距離,画素値から実際にどれだけ離れているか算出しその答えを返す.
                  
quintic_polynomials_planner : PythonRobotics/PathPlanning/QuinticPolynomialsPlannerから引用.

