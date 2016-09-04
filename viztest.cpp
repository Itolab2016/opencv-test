#include <opencv2/viz/vizcore.hpp>
#include <iostream>

using namespace cv;
using namespace std;

/**
 * * @function main
 * */
int main()
{
  /// Create a window
  viz::Viz3d myWindow("Viz Demo");

  /// Start event loop
  myWindow.spin();

  /// Event loop is over when pressed q, Q, e, E
  cout << "First event loop is over" << endl;

  /// Access window via its name
  viz::Viz3d sameWindow = viz::getWindowByName("Viz Demo");

  /// Start event loop
  sameWindow.spin();

  /// Event loop is over when pressed q, Q, e, E
  cout << "Second event loop is over" << endl;

  /// Event loop is over when pressed q, Q, e, E
  /// Start event loop once for 1 millisecond
  sameWindow.spinOnce(1, true);
  while(!sameWindow.wasStopped())
  {
    /// Interact with window

    /// Event loop for 1 millisecond
    sameWindow.spinOnce(1, true);
  }

  /// Once more event loop is stopped
  cout << "Last event loop is over" << endl;
  return 0;
}
