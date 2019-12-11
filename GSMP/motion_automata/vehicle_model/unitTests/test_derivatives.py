import sys
# Add the ptdraft folder path to the sys.path list
sys.path.append('../')

from parameters_vehicle2 import parameters_vehicle2
from vehicleDynamics_KS import vehicleDynamics_KS
from vehicleDynamics_ST import vehicleDynamics_ST
from vehicleDynamics_MB import vehicleDynamics_MB
import numpy

def test_derivatives():
    # test_derivatives - unit_test_function for checking whether the derivative
    # is computed correctly
    #
    # Syntax:  
    #    res = test_derivatives()
    #
    # Inputs:
    #    ---
    #
    # Outputs:
    #    res - boolean result (0/empty: test not passed, 1: test passed)
    #
    # Example: 
    #
    # Other m-files required: none
    # Subfunctions: none
    # MAT-files required: none
    #
    # See also: ---

    # Author:       Matthias Althoff
    # Written:      17-December-2017
    # Last update:  ---
    # Last revision:---

    #------------- BEGIN CODE --------------

    # initialize result
    res = []

    # load parameters
    p = parameters_vehicle2()
    g = 9.81 #[m/s^2]

    # set state values
    x_ks = [3.9579422297936526, 0.0391650102771405, 0.0378491427211811, 16.3546957860883566, 0.0294717351052816]
    x_st = [2.0233348142065677, 0.0041907137716636, 0.0197545248559617, 15.7216236334290116, 0.0025857914776859, 0.0529001056654038, 0.0033012170610298]
    x_mb = [10.8808433066274794, 0.5371850187869442, 0.0980442671005920, 18.0711398687457745, 0.1649995631003776, 0.6158755000936103, -0.1198403612262477, -0.2762672756169581, 0.0131909920269115, -0.0718483683742141, -0.3428324595054725, 0.0103233373083297, -0.0399590564140291, -0.0246468320579360, -0.0551575051990853, 0.5798277643297529, 0.0172059354801703, 0.0067890113155477, -0.0184269459410162, -0.0408207136116175, -1.0064484829203018, 0.0166808347900582, -0.0049188492004049, 53.6551710641082451, 50.7045242506744316, 56.7917911784219598, 200.7079633169796296, -0.0939969123691911, -0.0881514621614376]

    # set input: (accelerating and steering)
    v_delta = 0.15
    acc = 0.63*g
    u = [v_delta,  acc]

    # insert into vehicle dynamics to obtain derivatives
    f_ks = vehicleDynamics_KS(x_ks,u,p)
    f_st = vehicleDynamics_ST(x_st,u,p)
    f_mb = vehicleDynamics_MB(x_mb,u,p)

    # ground truth
    f_ks_gt = [16.3475935934250209, 0.4819314886013121, 0.1500000000000000, 5.1464424102339752, 0.2401426578627629]
    f_st_gt = [15.7213512030862397, 0.0925527979719355, 0.1500000000000000, 5.3536773276413925, 0.0529001056654038, 0.6435589397748606, 0.0313297971641291]
    f_mb_gt = [17.8820162482414098, 2.6300428035858809, 0.1500000000000000, 2.7009396644636450, 0.6158755000936103, 1.3132879472301846, -0.2762672756169581, -0.1360581472638375, -0.0718483683742141, 0.4909514227532223, -2.6454134031927374, -0.0399590564140291, 0.0486778649724968, -0.0551575051990853, 0.0354501802049087, -1.1130397141873534, 0.0067890113155477, -0.0886810139593130, -0.0408207136116175, 0.0427680029698811, -4.3436374104751501, -0.0049188492004049, 0.1142109377736169, 10.0527321757776047, 0.0436512393438736, 14.3044528684659404, 590.5602192640882322, -0.2105876149500405, -0.2126005780977984]

    # comparison    
    res.append(all(abs(numpy.array(f_ks) - numpy.array(f_ks_gt)) < 1e-14))
    res.append(all(abs(numpy.array(f_st) - numpy.array(f_st_gt)) < 1e-14))
    res.append(all(abs(numpy.array(f_mb) - numpy.array(f_mb_gt)) < 1e-14))
    
#    print(abs(numpy.array(f_ks) - numpy.array(f_ks_gt)))
#    print(abs(numpy.array(f_st) - numpy.array(f_st_gt)))
#    print(abs(numpy.array(f_mb) - numpy.array(f_mb_gt)))

    # obtain final result
    res = all(res)
    print('derivative test:')
    print(res)
    
    return res

    #------------- END OF CODE --------------
