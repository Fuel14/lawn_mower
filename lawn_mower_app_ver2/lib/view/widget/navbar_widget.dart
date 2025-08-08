import 'package:flutter/material.dart';
import 'package:lawn_mower_app_ver2/data/notifiers.dart';

class NavbarWidget extends StatelessWidget {
  const NavbarWidget({super.key});

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder(
      valueListenable: selectPageNotifier,
      builder: (context, selectedPage, child) {
        return NavigationBar(
          destinations: [
            NavigationDestination(
              icon: Icon(Icons.home), 
              label: 'Home'
            ),
            NavigationDestination(
              icon: Icon(Icons.bluetooth),
              label: 'BT terminal',
            ),
            NavigationDestination(
              icon: Icon(Icons.car_rental),
              label: 'Manual',
            ),
            NavigationDestination(
              icon: Icon(Icons.map), 
              label: 'GPS'
            ),
          ],
          onDestinationSelected: (value) {
            selectPageNotifier.value = value;
          },
          selectedIndex: selectedPage,
        );
      },
    );
  }
}
