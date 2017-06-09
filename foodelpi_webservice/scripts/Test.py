import yaml

stream = open('/home/pio-rosa/catkin_ws/src/food_delivery_pioneer/foodelpi_webservice/cfg/menu.yaml', 'r')
menus = yaml.load(stream)
for menu in menus:
    for menu_part, items in menu.iteritems():
        for item in items:
            for key, value in item.iteritems():
                print(key, ' -> ', value)
            print('\n')