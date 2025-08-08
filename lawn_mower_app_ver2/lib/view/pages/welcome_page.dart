import 'package:flutter/material.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';
import 'package:lawn_mower_app_ver2/view/widget/hero_widget.dart';
import 'package:lawn_mower_app_ver2/view/widget_tree.dart';

class WelcomePage extends StatelessWidget {
  const WelcomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            FittedBox(child: Text('Lawn Mowing by Elong Ma',style: KTextStyle.welcomTitleText,)),
            SizedBox(height: 20.0,),
            HeroWidget(),
            SizedBox(height: 20.0,),
            SizedBox(height: 50.0,),
            FilledButton(
              onPressed: () {
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(
                    builder: (context) {
                      return WidgetTree();
                    },
                  ),
                );
              },
              child: Text('Login'),
            ),
          ],
        ),
      ),
    );
  }
}
