cd spinnaker_tools
make clean
make || exit $?
cd ..

cd spinn_common
make clean 
make || exit $?
make install
cd ..

cd SpiNNFrontEndCommon/c_common/
cd front_end_common_lib/
make install-clean
cd ..
make clean
make || exit $?
make install 
cd ../..

cd SpiNNakerGraphFrontEnd/spinnaker_graph_front_end/examples/
make clean
make || exit $?

echo "completed"
