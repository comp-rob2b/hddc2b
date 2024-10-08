// SPDX-License-Identifier: LGPL-3.0
#include <check.h>

extern TCase *hddc2b_wheel_test(void);


int main(void)
{
    Suite *s = suite_create("core");
    suite_add_tcase(s, hddc2b_wheel_test());

    SRunner *sr = srunner_create(s);

    srunner_run_all(sr, CK_ENV);
    int nf = srunner_ntests_failed(sr);
    srunner_free(sr);

    return nf == 0 ? 0 : 1;
}
