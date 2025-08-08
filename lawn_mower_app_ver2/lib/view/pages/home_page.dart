import 'package:flutter/material.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';
import 'package:lawn_mower_app_ver2/view/widget/containerWidget.dart';
import 'package:lawn_mower_app_ver2/view/widget/hero_widget.dart';

class HomePage extends StatelessWidget {
  const HomePage({super.key});

  @override
  Widget build(BuildContext context) {
    double widthscreen =MediaQuery.of(context).size.width;
    return SingleChildScrollView(
      child: Center(
        child: Padding(
          padding: EdgeInsets.all(20.0),
          child: LayoutBuilder(
            builder: (context, BoxConstraints constraints) {
              return FractionallySizedBox(
                widthFactor: widthscreen > 500 ? 0.5 : 1.0,
                child: Column(
                  children: [
                    FittedBox(
                      child: Text(
                        'Rasicm with Elong Ma',
                        style: KTextStyle.welcomTitleText,
                      ),
                    ),
                    SizedBox(height: 20.0),
                    HeroWidget(),
                    SizedBox(height: 20.0),
                    Containerwidget(
                      title: 'Black',
                      description:
                          'Black People, Theft, Run fast, Belong in jail if not fast enough',
                    ),
                    Containerwidget(
                      title: 'Yellow',
                      description:
                          'Eat dog, have no daughter, small PP, good at math',
                    ),
                    Containerwidget(
                      title: 'White',
                      description: 'So fat, Woke af',
                    ),
                  ],
                ),
              );
            },
          ),
        ),
      ),
    );
  }
}
