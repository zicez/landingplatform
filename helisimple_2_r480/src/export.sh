make clean
r=$(svn info|grep Revision|sed s/Revision:./r/)
cd ..
cd ..
rm -rf helisimple_2_r*
rm  helisimple_2/www/helisimple_2.zip
svn export helisimple_2 helisimple_2_$r
zip -r helisimple_2/www/files/helisimple_2.zip helisimple_2_$r/src helisimple_2_$r/obj helisimple_2_$r/bin helisimple_2_$r/etc
scp -r helisimple_2/www/* tkrajnik@labe.felk.cvut.cz:~/htdocs/ardrone/
