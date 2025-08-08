import 'package:flutter/material.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';
import 'package:lawn_mower_app_ver2/data/notifiers.dart';
import 'package:lawn_mower_app_ver2/view/pages/welcome_page.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  runApp(const MyApp());
}

void nahee() {
  debugPrint('PongmuengTai');
}

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State<MyApp> createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  @override
  void initState() {
    initThemeMode();
    super.initState();
  }

  void initThemeMode() async{
    final SharedPreferences pref = await SharedPreferences.getInstance();
    final bool? repeat = pref.getBool(KConstant.isLight);
    screenLightStateNotifier.value = repeat ?? false;
  }
  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder(
      valueListenable: screenLightStateNotifier,
      builder: (context, screenlightstate, child) {
        return MaterialApp(
          debugShowCheckedModeBanner: false,
          theme: ThemeData(
            colorScheme: ColorScheme.fromSeed(
              seedColor: Colors.teal,
              brightness: screenlightstate == true
                  ? Brightness.light
                  : Brightness.dark,
            ),
          ),
          home: WelcomePage(),
        );
      },
    );
  }

  
}
