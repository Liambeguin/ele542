echo_header() {
	cat << EOF
/*
 * This file is part of Lab1 ELE542
 *
 * "THE BEER-WARE LICENSE" (Revision 42):
 *  As long as you retain this notice you can do whatever you want with this
 *  stuff. If we meet some day, and you think this stuff is worth it,
 *  you can buy us a beer in return.
 *  If you use this at ETS, beware of the shool's policy against copying
 *  and fraud.
 *
 *   Filename : %%FILENAME%%
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

EOF
}

do_add_header() {
	file="$(basename $1)"
	echo -- $file
	if grep -q BEER $1 ; then
		echo --- already processed
	else
		echo_header > tmp
		cp $1 $1.tmp
		cat tmp $1.tmp > $1
		sed -i "s/%%FILENAME%%/"$file"/" $1
		rm $1.tmp tmp
	fi
}

for i in $(find -type f -name '*.[ch]'); do
	do_add_header $i
done

