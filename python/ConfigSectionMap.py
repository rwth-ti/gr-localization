import ConfigParser
def ConfigSectionMap(Config, section):
    dict1 = {}
    options = Config.options(section)
    for option in options:
        try:
            try:
                dict1[option] = float(Config.get(section, option).split("#")[0])
            except:
                dict1[option] = Config.get(section, option).split("#")[0].strip()
            if dict1[option] == -1:
                DebugPrint("skip: %s" % option)
        except:
            print("exception on %s!" % option)
            dict1[option] = None
    return dict1
