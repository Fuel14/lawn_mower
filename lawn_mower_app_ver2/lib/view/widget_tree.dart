import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';
import 'package:lawn_mower_app_ver2/data/notifiers.dart';
import 'package:lawn_mower_app_ver2/view/pages/GPS.dart';
import 'package:lawn_mower_app_ver2/view/pages/Manual.dart';
import 'package:lawn_mower_app_ver2/view/pages/bluetooth_page.dart';
import 'package:lawn_mower_app_ver2/view/pages/home_page.dart';
import 'package:lawn_mower_app_ver2/view/pages/setting_page.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'widget/navbar_widget.dart';


class WidgetTree extends StatelessWidget {
  const WidgetTree({super.key});

  get _flutterBlueClassicPlugin => FlutterBlueClassic();
  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: ValueListenableBuilder(
        valueListenable: readSubscriptionNotifier,
        builder: (context, readSubscription, child) {
          return ValueListenableBuilder(
            valueListenable: bluetoothDeviceAdressNotifier,
            builder: (context, connection, child) {
              return Scaffold(
                appBar: AppBar(
                  title: Text('Lawn Mower App'),
                  centerTitle: true,
                  actions: [
                    // Light Dark Mode switch Icon
                    IconButton(
                      onPressed: () async {
                        screenLightStateNotifier.value =
                            !screenLightStateNotifier.value;
                        final SharedPreferences pref =
                            await SharedPreferences.getInstance();
                        await pref.setBool(
                          KConstant.isLight,
                          screenLightStateNotifier.value,
                        );
                      },
                      icon: ValueListenableBuilder(
                        valueListenable: screenLightStateNotifier,
                        builder: (context, screenlightstate, child) {
                          return Icon(
                            screenlightstate == true
                                ? Icons.dark_mode
                                : Icons.light_mode,
                          );
                        },
                      ),
                    ),

                    // To setting page icon
                    IconButton(
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute(
                            builder: (context) {
                              return SettingPage();
                            },
                          ),
                        );
                      },
                      icon: Icon(Icons.settings),
                    ),
                    ValueListenableBuilder(
                      valueListenable: bluetoothDeviceDataNotifier,
                      builder: (context, result, child) {
                        return IconButton(
                          onPressed: () async {
                            if (connection == null) {
                              ScaffoldMessenger.maybeOf(context)?.showSnackBar(
                                const SnackBar(
                                  content: Text("No selecting device"),
                                  duration: Duration(seconds: 1),
                                ),
                              );
                            } else {
                              bluetoothDeviceConnectingStatusNotifier.value =
                                  !connection!.isConnected;
                              if (connection?.isConnected == false) {
                                try {
                                  connection = await _flutterBlueClassicPlugin
                                      .connect(result?.address);
                                  bluetoothDeviceAdressNotifier.value =
                                      connection;
                                } catch (e) {
                                  if (kDebugMode) print(e);
                                  connection?.dispose();
                                  ScaffoldMessenger.maybeOf(
                                    context,
                                  )?.showSnackBar(
                                    const SnackBar(
                                      content: Text(
                                        "Error connecting to device",
                                      ),
                                      duration: Duration(seconds: 1),
                                    ),
                                  );
                                }
                              } else {
                                try {
                                  connection?.dispose();
                                  readSubscription?.cancel();
                                  bluetoothDeviceAdressNotifier.value =
                                      connection;
                                  bluetoothDeviceDataNotifier.value = result;
                                  subscribtionStateNotifier.value = false;
                                } catch (e) {
                                  if (kDebugMode) print(e);
                                  connection?.dispose();
                                  ScaffoldMessenger.maybeOf(
                                    context,
                                  )?.showSnackBar(
                                    const SnackBar(
                                      content: Text(
                                        "Error disconnect to device",
                                      ),
                                    ),
                                  );
                                }
                              }
                            }
                          },
                          icon: ValueListenableBuilder(
                            valueListenable:
                                bluetoothDeviceConnectingStatusNotifier,
                            builder:
                                (context, bluetoothConnetionStatus, child) {
                                  return Icon(
                                    bluetoothConnetionStatus == true
                                        ? Icons.bluetooth_connected
                                        : Icons.bluetooth_disabled,
                                    color: bluetoothConnetionStatus == true
                                        ? Colors.blue
                                        : Colors.grey,
                                  );
                                },
                          ),
                        );
                      },
                    ),
                  ],
                ),

                //Body pages
                body: ValueListenableBuilder(
                  valueListenable: selectPageNotifier,
                  builder: (context, selectedPage, child) {
                    return _buildPage(selectedPage);
                  },
                ),

                //Bottom Navigation
                bottomNavigationBar: NavbarWidget(),
              );
            },
          );
        },
      ),
    );
  }

  Widget _buildPage(int selectedPage) {
    switch (selectedPage) {
      case 0:
        return const HomePage();
      case 1:
        return ValueListenableBuilder(
          valueListenable: bluetoothDeviceAdressNotifier,
          builder: (context, connection, child) {
            return BluetoothPage(connection: connection!);
          },
        );
      case 2:
        return ValueListenableBuilder(
          valueListenable: bluetoothDeviceAdressNotifier,
          builder: (context, connection, child) {
            return ManualPage(connection: connection!);
          },
        );

      case 3:
        return  ValueListenableBuilder(
          valueListenable: bluetoothDeviceAdressNotifier,
          builder: (context, connection, child) {
            return gpsPage(connection: connection!);
          },
        );
      default:
        return const HomePage(); // Fallback
    }
  }
}
