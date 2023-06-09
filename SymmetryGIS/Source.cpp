﻿#include <pch.h>
#include "RibbonUI.h"
#include <UxTheme.h>
#include <filesystem>
#include <vector>

#include <GF/Extensions/Extensions.h>

#pragma comment(lib, "version.lib")
#pragma comment (lib, "UxTheme.lib")
#pragma comment(lib, "EzLasLib.lib")
using namespace GemmaFusion;

std::tuple<RasterLayer, RasterObject> GetClassificationMask(const std::string& classifiedLasPath, double resolution)
{
	// Load classified lidar
	LasContainer lasContainer;
	auto lasObject = lasContainer->AddLas(classifiedLasPath);

	// Create spatial point index for faster point search
	lasContainer->IndexLas(lasObject);

	// Create empty raster layer out of las file and then create raster object with provided resolution
	RasterLayer lasRaster = lasContainer->CreateLasRasterLayer(lasObject);
	RasterObject lasClassification = lasRaster->CreateRasterObject(resolution);

	// Fill raster object with classification data
	auto image = lasClassification.GetImage();
	image.SetAllPixels(0.0f);										// Set all values to 0
	GF::Processing::Features::LAS::FillGridWithLidar(
		lasContainer,												// las container to be used for point extraction. This function can extract points from multiple las files
		GF::Processing::Features::LAS::LasProperty::Classification, // Point attribute to extract into image
		{ 6 },											// Classifications to extract
		lasClassification);											// Target raster object into which values are written

	return { lasRaster, lasClassification };
}

void ClosingByReconstruction(RasterObject object, int winSize)
{
	auto grid = GF::Extensions::Interop::GetGrid(object);
	grid.ClosingByReconstruction(winSize);
	GF::Extensions::Interop::CopyValues(grid, object);
}

std::vector<Point3d> GetBuildingPoints(const std::string& classifiedLasPath, ShapeObject buildingOutline)
{
	// Open las file
	LasObject lasObject(classifiedLasPath);

	// Prepare las native boundingbox
	BoundingBox3i64 bbox;
	bbox.Expand(lasObject->ReverseTransform(buildingOutline->GetBoundingBox2d().Min.cast<Point3d>()));
	bbox.Expand(lasObject->ReverseTransform(buildingOutline->GetBoundingBox2d().Max.cast<Point3d>()));

	// Get composed polygons (multipolygon structure with holes)
	auto polygons = buildingOutline->GetComposedPolygons();

	// Filter out points that belong to the building
	std::vector<Point3d> points;
	lasObject->Filter(bbox, true, [&](LAS::const_Point point) {
		auto ptNative = Point3i64(point.X(), point.Y(), point.Z());//.convert_from(point.XYZ()).cast<Point3i64>();
		auto ptTransformed = lasObject->TransformCoord(ptNative);
		for (auto& polygon : polygons) {
			if (!polygon.Contains2d(ptTransformed)) continue;
			points.push_back(ptTransformed);
		}
	});

	return points;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR cmdArgs, int nShowWnd)
{
	//
	// Init application
	//

	// Init logger (debug output)
	AppLogger()->SetLogLevel(0);
	AppLogger()->EnableLine(true);// Output line
	AppLogger()->EnableTime(true);// Output line
	AppLogger()->EnableFunction(true);// Output function name
	AppLogger()->EnableLogging(true);
	

	APPRESULT_INFORMATION("SymmetryGIS %d", 6);

	// Init
	winrt::init_apartment(winrt::apartment_type::single_threaded);		// Init COM apartment
	SetThreadPerMonitorDpiAwarenessContext();							// This application is HiDPI aware and can handle scaling on its own
	g_ApplicationState.GraphicsProfile = GraphicsProfile::HighQuality;  // Global variable to toggle graphics quality (will be removed in future releases)
	InitializeFrequent();												// Init legacy STEZA "security" layer which decodes commonly used constants used by camera and renderers (DEV MODE)

	ApplicationInstance applicationInstance(hInstance);	// Current application instance with event loop
		
	// Init main spatial projection and transformations
	Epsg epsgStore;
	Crs wgs84 = epsgStore.GetCrs(4326);
	Crs eps3794 = epsgStore.GetCrs(3794);
	CrsTransformation geographicTransform(eps3794, wgs84);
	CrsTransformation projectedTransform(wgs84, eps3794);

	GdiPlusInstance gdiPlusInstance; // Init GDI+ (Required for png, jpg which are primarly used in MapLayerRenderer)
	
	Window mainWindow(L"Symmetry GIS", 1300, 700);		// Main window
	//
	// Init UI 
	// 
	
	// Riborn menu
	RibbonFramework ribbon(mainWindow);
	EmptyLayout ribbonLayout;

	// Canvas (Main data drawing)
	GraphicsInfrastructure gi;
	GraphicsDevice9 deviceD3D9;	// D3D9 graphics device needed for canvas
	GraphicsDevice11 deviceD3D11 = gi.CreateDevice(::Graphics::DebugDevice::None, false); // D3D11 graphics device needed for canvas
	CanvasControl canvas(deviceD3D9, deviceD3D11, mainWindow);	// 2D Canvas for displaying shapes, rasters and lidar
	
	// Status bar
	StatusBar statusBar(mainWindow->GenerateCmdId(), mainWindow);
	//statusBar->AddLabel("projection", 350, std::format("🌐 EPSG:{} ({})", utm32n.GetEpsg(), utm32n.GetName()));
	statusBar->AddLabel("projection", 200, std::format("🌐 EPSG:{}\n({})", eps3794.GetEpsg(), eps3794.GetName()));
	statusBar->AddLabel("coordinatesIcon", 20, "📍");
	statusBar->AddLabel("coordinates", 140, "CX");
	statusBar->AddLabel("longlat", 140, "GX");
	statusBar->AddLabel("zoom", 100, "Zoom");

	// Prepare main layout for window (binds WM_SIZE handler to the main window)
	StackLayout mainLayout(mainWindow, LayoutType::Vertical);
	ribbon->SetOnViewChange([&]() {
		mainLayout->ModifyLayout(ribbonLayout, UI::SizeType::FixedPixels, ribbon->GetRibbonHeight());
	});
	mainLayout->AddLayout(ribbonLayout); // Top ribbon
	mainLayout->AddControl(canvas);  // Canvas will use rest of the free space
	mainLayout->AddControl(statusBar, SizeType::FixedDips, 50); // Bottom control (statusbar) will have height of 25 dips




	//bool enableContext = true;
	//
	//auto l_enabled = [&]() -> bool
	//{
	//	return !enableContext;
	//};

	//auto l_invalidate_props = [&]
	//{
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_BUTTON);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_COLOR);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_CHECKBOX);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_TEXT);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_NUMERIC);
	//	//ribbon->InvalidateProperty(IDC_CMD_TAB_CONTEXT_GROUP);
	//};


	//
	// Init rendering and containers
	//
	
	// Get 3D camera from canvas
	Camera m_Camera = canvas->GetCamera(); 
	m_Camera->SetZ(-300000.0f);

	// Setup georeferencer (transforms georeferenced points into 3D world space)
	auto m_Georeferencer = Core::make_ptr<WorldGeoreferencer>(m_Camera);
	m_Georeferencer->SetBaseScale({ 0.5, -0.5, -0.5 });
	//georeferencer->SetBasePoint({ 1741650, 5870027 });

	// Prepare shape renderer
	auto shapeRenderer = Core::make_ptr<ShapeRenderer>(deviceD3D9, m_Camera, m_Georeferencer);
	shapeRenderer->SetDpi(mainWindow->GetDpi());
	// Prepare ShapeLayer container and assign renderers. ShapeLayer is automatically added to all renderers when inserted into a drawing
	ShapeDrawing shapeDrawing;
	shapeDrawing->Initialize({ shapeRenderer });
	shapeDrawing->SetTargetCrs(eps3794);
	// Prepare shape loader which prepares shapelayer for rendering in the background
	ShapeLoader shapeLoader;
	shapeLoader->AddLayerRenderer(shapeRenderer);
	shapeLoader->Start();

	// Prepare raster renderer
	auto rasterRenderer = Core::make_ptr<MonoRasterLayerRenderer>(deviceD3D9, m_Camera, m_Georeferencer);
	// Prepare RasterLayer container and assign renderers. RasterLayer is automatically added to all renderers when inserted into a drawing	
	RasterDrawing rasterDrawing;
	rasterDrawing->Initialize({ rasterRenderer });
	rasterDrawing->SetTargetCrs(eps3794);

	// Prepare map renderer
	MapLayerRenderer mapRenderer(deviceD3D9, m_Camera, m_Georeferencer);
	mapRenderer->EnableTextureFiltering(true);
	mapRenderer->SwapRedBlueColors(true); // 
	mapRenderer->SetTargetCrs(eps3794);

	// LasContainer is mixture of ShapeLayer and RasterLayer (uses both of them to achieve 2D rendering)
	LasContainer lasContainer;
	lasContainer->SetTargetCrs(eps3794);
	// Add las container shapelayer to the renderer and set it as support layer
	ShapeLayer lasShapeLayer = lasContainer->GetLasLayer();
	//lasShapeLayer->GetStyle().FillSymbol().Show(false); // Disable shape fill
	shapeRenderer->CreateLayerObject(lasShapeLayer);

	TmsFetcher basemapFetcher("https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}.png");
	MapLayer basemapObject = mapRenderer->CreateLayerObject(basemapFetcher.as<TileFetcher>());

	// Current thread work dispatcher (captures current thread via RAII)
	ThreadDispatcher uiThread;
	// Prepare thread for background processing (lidar rendering or anything else)
	ThreadWorker processingThread("Processing thread", new_thread, Platform::Infinite());
		
	
	CanvasHelper canvasHelper(m_Camera, m_Georeferencer, shapeDrawing);

	applicationInstance->GetWorker().Post([&] { canvasHelper.PanToPoint(Geometry::Point3d(499500, 113000, 0)); });
	applicationInstance->GetWorker().Post([&] { std::filesystem::remove_all("tmp\\"); });
	
	///// For drawing
	ShapeLayer m_Selection = ShapeLayer(SpecificGeometryType::PolygonZ);
	m_Selection->GetStyle().PointSymbol().Show(false);
	m_Selection->SetName("Selection");
	shapeRenderer->CreateLayerObject(m_Selection);
	shapeRenderer->ChangeToSupportLayer(m_Selection);
		
	std::string m_LASFilename;
	Geometry::Point3d m_SelectionPointDown;
	BoundingBox2d SelectionBB;
	std::vector<GF::Spatial::Shape::FeatureIdentifyObject> SelectedObjs;
	//
	// Prepare initial data for visualization
	//
	
	// Define new custom colormap which will perform hillshade operation on lidar and sample raster object
	auto l_hillshade_colormap = [](Core::Ptr<GF::Spatial::Raster::impl::RasterObject> object, TiffTools::TiffImage<TiffTools::PixelDataRGBA8> outputImage) mutable
	{
		TiffTools::TiffImage<float> image = object->GetImage();
		auto grid = GF::Extensions::Interop::GetGrid(object.Get());
		grid.ApplyHillShade(45.0, 315.0);

		for(int y = 0; y < image.GetHeight(); y++)
		{
			for(int x = 0; x < image.GetWidth(); x++)
			{
				Core::Byte value = static_cast<Core::Byte>(grid.GetValue(x, y) * 255.0);
				TiffTools::PixelDataRGBA8 rgba{value, value, value, 255};
				outputImage.SetPixel(x, y, rgba);
			}
		}
	};

	//color mapping
	auto SetColorFromScore = [](float score, auto lasObj) mutable
	{
		APPRESULT_DEV_INFORMATION("%f", score);
		auto col = score / 10000;
		//auto val = col / 255;

		col = std::min(col , 1.0f);


		lasObj->SetStyle(ShapeStyle().GetStyle().SetColor({col,1.0f - col,0.0f,1.0f}));
	};

	//reading symetrie file
	auto readSymetrieFile = [](std::string filename, Utils::Plane& plane) mutable
	{
		std::ifstream file(filename);
		//file.open(filename, std::ios::in);

		if(!file.is_open())
		{
			throw std::exception("File not opened!");
			return;
		}

		std::string line;
		std::vector<double> list;
		while(std::getline(file, line, ';'))
		{
			list.push_back(atof(line.c_str()));
		}

		plane.a = (float)list[0];
		plane.b = (float)list[1];
		plane.c = (float)list[2];
		plane.d = (float)list[3];

		file.close();
	};

	auto GetGeoreferencedScreenCoordinates = [&](const Geometry::Point2i& screenCoordinates)
	{
		auto world = m_Camera->ScreenToWorldCoordinates(screenCoordinates);
		auto geo = m_Georeferencer->WorldToGeoreferencedCoordinates(world.cast<Geometry::Point3d>());
		geo.Z = 0;
		return geo;
	};	

	//
	// Event handlers
	//
	canvas->SetDrawEvent([&](Camera camera)
	{
		deviceD3D9->GetD3D9Device()->SetRenderState(D3DRS_ZENABLE, FALSE);// Disable ZBuffer
		// Render layers
		mapRenderer->Draw();
		rasterRenderer->Draw();
		shapeRenderer->Update();
		shapeRenderer->Draw();
		return true;
	});

	auto lastUpdateStatus = std::chrono::high_resolution_clock::now();
	canvas->SetMouseEvent(MouseEventType::MouseMove, [&](const Mouse& mouse) 
	{
		auto currPos = GetGeoreferencedScreenCoordinates(mouse.CurrentPosition2());

		auto updateStatus = std::chrono::high_resolution_clock::now();
		if(std::chrono::duration_cast<std::chrono::milliseconds>(updateStatus - lastUpdateStatus).count() > 64)
		{
			auto latLon = geographicTransform.TransformCoordinate(currPos);

			// This file is saved as UTF-8 with BOM which means utf8 is supported by GF Apis (std::string)
			statusBar->SetItemText("coordinates", Core::String::Format("%.2f m\n%.2f m", currPos.X, currPos.Y));
			statusBar->SetItemText("longlat", Core::String::Format("%f°\n%f°", latLon.X, latLon.Y));
			statusBar->SetItemText("zoom", Core::String::Format("🔎 %.2f", abs(m_Camera->GetPosition2().Z)));

			lastUpdateStatus = updateStatus;
		}

		if(mouse[MK_LBUTTON | MK_CONTROL])
		{
			Geometry::BoundingBox2d bb;
			bb.Expand2d(m_SelectionPointDown);
			bb.Expand2d(currPos);

			if (m_Selection.size() > 0)
			{
				auto obj = m_Selection[0];
				if (obj[0].Count() > 0)
				{
					auto lock = obj.Lock();
					obj[0].Point(0) = { bb.Min.X, bb.Min.Y, 0.0 };
					obj[0].Point(1) = { bb.Max.X, bb.Min.Y, 0.0 };
					obj[0].Point(2) = { bb.Max.X, bb.Max.Y, 0.0 };
					obj[0].Point(3) = { bb.Min.X, bb.Max.Y, 0.0 };
					obj[0].Point(4) = { bb.Min.X, bb.Min.Y, 0.0 };
					obj->UpdateLayerBounds();
				}
			}
		}
		shapeRenderer->Update(true);
	});

	canvas->SetMouseEvent(MouseEventType::LButtonDown, [&](const Mouse& mouse) 
	{

		if(mouse[MK_CONTROL])
		{
			m_SelectionPointDown = GetGeoreferencedScreenCoordinates(mouse.CurrentPosition2());
			
			m_Selection->ClearLayer();
			ShapePart part(5);
			ShapeObject select = m_Selection->CreateShapeObject({ part }, {});
		}
		else
		{
			canvas->SetCursor(MouseCursor::SizeAll);
		}
	});

	canvas->SetMouseEvent(MouseEventType::LButtonUp, [&](const Mouse& mouse) 
	{
		auto currPos = mouse.CurrentPosition2();

		if(mouse[MK_CONTROL])
		{//create bb from selection
			auto secondPoint = GetGeoreferencedScreenCoordinates(currPos);
			if (secondPoint == m_SelectionPointDown)
			{//picking polygone
				auto layer = shapeDrawing->FindLayerByName("buildings");
				if(layer!= nullptr)
				{
					layer->DeselectAllShapes();
					for(auto shape : layer->GetIntersectedObjects(secondPoint))
					{
						shape.GetShapeObject()->Select(true);
					}
					//SelectedObjs = layer->GetIntersectedObjects(secondPoint);
					//SelectedObjs[0].GetShapeObject()->Select(true);
				}
			}
			else
			{// selecting area
				if (m_Selection.size() > 0)
				{
					auto layer = shapeDrawing->FindLayerByName("buildings");
					if(layer != nullptr)
					{
						layer->DeselectAllShapes();
					}
					auto obj = m_Selection[0];
					SelectionBB = m_Selection->GetBoundingBox();
					SelectionBB = obj->GetBoundingBox2d();
				}
			}
		}
		shapeDrawing->Redraw(m_Selection);
		shapeDrawing->Redraw();

		canvas->SetCursor(MouseCursor::Arrow);
	});

	canvas->SetMouseEvent(MouseEventType::RButtonUp, [&](const Mouse& mouse) {
		ContextMenu canvasContextMenu;
		auto [x, y] = canvas->GetScreenCoordinates(static_cast<int>(mouse.CurrentPosition().x), static_cast<int>(mouse.CurrentPosition().y));
		canvasContextMenu->Open(canvas, x, y);
	});

	canvas->SetMouseWheelEvent([&](const Mouse& mouse, int zDelta) {
		shapeRenderer->Update(true);
		auto updateStatus = std::chrono::high_resolution_clock::now();
		if(std::chrono::duration_cast<std::chrono::milliseconds>(updateStatus - lastUpdateStatus).count() > 64)
		{
			statusBar->SetItemText("zoom", Core::String::Format("🔎 %.2f", abs(m_Camera->GetPosition2().Z)));
			lastUpdateStatus = updateStatus;
		}
	});

	mainWindow->AddEventHandler(WM_CLOSE, [&](HWND hWnd, WPARAM wParam, LPARAM lParam, LRESULT& lResult)
	{
		applicationInstance->Exit(0);
		return true;
	});

	mainWindow->AddEventHandler(WM_DPICHANGED, [&](HWND hWnd, WPARAM wParam, LPARAM lParam, LRESULT& lResult)
	{
		shapeRenderer->SetDpi(mainWindow->GetDpi());
		return false;
	});

	// Button click
	ribbon->SetOnCommandExecute(ID_EXIT, [&]()
	{
		std::filesystem::remove_all("tmp\\");
		//APPRESULT_DEV_INFORMATION("Exit");
		applicationInstance->Exit(0);
	});
	
	ribbon->SetOnCommandExecute(ID_CLEARALL,[&]{
			shapeDrawing->Clear();
			rasterDrawing->Clear();
			lasContainer->Clear();
			m_Selection->ClearLayer();
	});

	ribbon->SetOnCommandExecute(ID_SWITCHPROJ, [&]{
		CrsPickerWindow crsPicker;
		if(auto epsgCode = crsPicker.GetEpsg(true))
		{
			Crs oldCrs = shapeDrawing->GetTargetCrs();
			Crs newCrs = epsgStore.GetCrs(epsgCode.value());
			if(newCrs.GetType() != CrsType::Projected)
			{
				MessageBoxW(0, L"Only projected crs types are supported", L"Error", MB_ICONERROR);
				return;
			}
			shapeDrawing->SetTargetCrs(newCrs);
			rasterDrawing->SetTargetCrs(newCrs);
			mapRenderer->SetTargetCrs(newCrs);
			lasContainer->SetTargetCrs(newCrs);
			projectedTransform = CrsTransformation(wgs84, newCrs);
			geographicTransform = CrsTransformation(newCrs, wgs84);
			statusBar->SetItemText("projection", std::format("🌐 EPSG:{} ({})", newCrs.GetEpsg(), newCrs.GetName()));

			// Retransform current location
			CrsTransformation transformation(oldCrs, newCrs);
			auto currentGeoreferenced = canvasHelper.GetCameraGeoreferencedCoordinates();
			currentGeoreferenced.Z = 0;
			auto newGeoreferenced = transformation.TransformCoordinate(currentGeoreferenced);
			canvasHelper.PanToPoint(newGeoreferenced);
			canvasHelper.RecenterGrid();
		}
	});

	//lasObject->Filter(bboxi, true, [&](const ::LAS::const_Point& lasPoint, uint64_t pointIndex)
	//				  {
	//					  Point3i64 pti{lasPoint.X(), lasPoint.Y(), lasPoint.Z()};
	//					  Point3d ptd = lasObject->TransformCoord(pti);
	//					  if(!Utility::PointInPolygon(ptd, boundary))
	//					  {
	//						  return;
	//					  }
	//					  int classification = (int)lasPoint.Classification();
	//					  if(!(classification >= 0 && classification < (int)m_S
	//						   ries.size()))
	//					  {
	//						  return;
	//					  }
	//					  // Reproject point to the target crs
	//					  if(pointTransform != nullptr)
	//					  {
	//						  ptd = pointTransform.TransformCoordinate(ptd);
	//					  }
	//					  LasPointInfo i{pointIndex, ptd, lasObject};
	//					  m_PointCache[classification].emplace_back(i);
	//				  });

	auto RenderWholeLas = [&]() mutable
	{
		// Submit lidar rendering to background thread
		processingThread.Post([uiThread, lasContainer, rasterDrawing, l_hillshade_colormap]() mutable
		{
			for (int i = 0; i < lasContainer->CountLasObjects(); i++) //for(auto& addedLas : added)
			{
				auto addedLas = lasContainer->GetLasObject(i);
				auto raster = lasContainer->CreateLasRasterLayer(addedLas);
				lasContainer->GenerateLasRasterObjects(raster, 1.0, GenerateLasRasterObjectProperties(true, false, { 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 }));
				// Submit rendered raster to the ui thread
				uiThread->Post([raster, rasterDrawing, l_hillshade_colormap]() mutable
				{
					raster[0]->SetCustomColorMapKey("Hillshade");
					raster[0]->SetCustomColorMapHandler(l_hillshade_colormap);
					rasterDrawing->AddRasterLayer(raster);
				});
			}
		});
	};

	auto RenderOnlyHouses = [&]() mutable
	{
		//// Submit lidar rendering to background thread
		processingThread.Post([uiThread, lasContainer, rasterDrawing, l_hillshade_colormap]() mutable
		{
			for (int i = 0; i < lasContainer->CountLasObjects(); i++)
				//for(auto& addedLas : added)
			{
				auto addedLas = lasContainer->GetLasObject(i);
				auto raster = lasContainer->CreateLasRasterLayer(addedLas);
				lasContainer->GenerateLasRasterObjects(raster, 1.0, GenerateLasRasterObjectProperties(true, false, { 6 }));
				// Submit rendered raster to the ui thread
				uiThread->Post([raster, rasterDrawing, l_hillshade_colormap]() mutable
				{
					raster[0]->SetCustomColorMapKey("Hillshade");
					raster[0]->SetCustomColorMapHandler(l_hillshade_colormap);
					rasterDrawing->AddRasterLayer(raster);
				});
			}
		});
	};

	ribbon->SetOnCommandExecute(ID_LOADDATA, [&] {
		//APPRESULT_DEV_INFORMATION("Load Data");

		FilePicker picker;
		auto las = picker.AddExtensionFilter("LiDAR files", "*.las");
		if(auto result = picker.OpenDialog("Open LiDAR file"))
		{
			shapeDrawing->Clear();
			rasterDrawing->Clear();
			lasContainer->Clear();

			m_LASFilename = result.value().Path();			
			//APPRESULT_DEV_INFORMATION(result.value().Path());
			//m_Data.push_back(result.value().Path());
			auto lasObj = lasContainer->AddLas(m_LASFilename);
			
			canvasHelper.ZoomToBoundingBox(lasObj->GetBoundingBox2d());
			//auto lay = lasContainer->GetLasLayer();

			RenderWholeLas();
			//		SetColorFromScore(Symmerty, lasObj);
			//		bb.ExpandByBoundingBox2d(lasObj->GetBoundingBox2d());
			
			//		added.push_back(lasObj);


			
		}

		//FilePicker picker;
		//auto las = picker.AddExtensionFilter("LiDAR files", "*.las");
		//if (auto result = picker.OpenMultipleDialog("Open lidar files"))
		//{
		//	std::vector<ShapeObject> added;
		//	std::vector<Utils::Plane> planes;
		//	Geometry::BoundingBox2d bb;
		//	for (auto& file : result.value().Path())
		//	{
		//		// Converting LAs file to point clound file format
		//		auto args = fmt::format("las2txt.exe -i {} --column-delimiter \" \" --line-format x y z --overwrite-files 1 --file-extension pc -o tmp\\", file);
		//		//APPRESULT_DEV_INFORMATION("%s", args);
		//		auto pi = Platform::Windows::Process::CreateProcessInstance("las2txt\\las2txt.exe", args);// , Platform::Interfaces::StandardIO::RedirectInputOutput);
		//		pi->Start();
		//		pi->Await();
		//		//std::string output = Platform::Windows::Process::GetProcessOutput(pi);
		//		//prepering point cloud file name 
		//		auto pos = file.find_last_of("\\");
		//		std::string pcFile(file, pos + 1); pcFile.insert(0, "tmp\\"); pcFile.replace(pcFile.size() - 3, 3, "pc");
		//		//APPRESULT_DEV_INFORMATION("%s", pcFile);
		//		
		//		// Calculating Symmetrie on the pointcloud
		//		auto argsym = fmt::format("SymmetryDetector.exe {} {}", pcFile, false); 
		//		//APPRESULT_DEV_INFORMATION("%s", argsym);
		//		auto piSym = Platform::Windows::Process::CreateProcessInstance("SymmetryDetector\\SymmetryDetector.exe", argsym, Platform::Interfaces::StandardIO::RedirectInputOutput);
		//		piSym->Start();//std::format()
		//		piSym->Await();
		//		std::string symRes = Platform::Windows::Process::GetProcessOutput(piSym);
		//		//APPRESULT_DEV_INFORMATION("%s", symRes);
		//		// Converting Symmetrie result into data
		//		// Time characteristic
		//		auto bTime = symRes.find("Time:")+6;
		//		auto eTime = symRes.find("\n", bTime);
		//		auto strTime = symRes.substr(bTime, eTime - bTime-1); 
		//		// Plane characteristic
		//		auto bPlane = symRes.find("The symmetry plane:")+20;
		//		auto ePlane = symRes.find("\n", bPlane);
		//		auto strPlane = symRes.substr(bPlane, ePlane - bPlane-1);
		//		std::vector<std::string> tokens;
		//		std::stringstream ss(strPlane);
		//		std::string intermediate;
		//		while (getline(ss, intermediate, ';'))
		//		{
		//			tokens.push_back(intermediate);
		//		}
		//		Utils::Plane plane;
		//		plane.a = (float)atof(tokens[0].c_str());
		//		plane.b = (float)atof(tokens[1].c_str());
		//		plane.c = (float)atof(tokens[2].c_str());
		//		plane.d = (float)atof(tokens[3].c_str());
		//		// Symetrie measure characteristic
		//		auto bSymmerty = symRes.find("Symmetry measure:")+18;
		//		auto eSymmerty = symRes.find("\n", bSymmerty);
		//		auto strSymmerty = symRes.substr(bSymmerty, eSymmerty - bSymmerty-1);
		//		/*auto dot = strSymmerty.find(',');
		//		strSymmerty.replace(strSymmerty.find(','), 1, ".");*/
		//		auto Symmerty = std::stof(strSymmerty);
		//		
		//		
		//		planes.push_back(plane);
		//		if(plane.c == 0.0)
		//		{
		//			// Calculating the Symmetrie Line to draw on the map 
		//			auto bb = added.back()->GetBoundingBox2d();
		//			Utils::BoundingBox6f BB((float)bb.Min.X, (float)bb.Min.Y, 0.f, (float)bb.Max.X, (float)bb.Max.Y, 0.f);
		//			Utils::Plane P = Utils::Plane(BB.GetBottomBackLeft(), BB.GetBottomBackRight(), BB.GetBottomFrontLeft());
		//			auto ray = Utils::Intersects(plane, P);
		//			float kao = Utils::DotProd(BB.Min + ray.o, ray.d);
		//			auto o = ray.o + ray.d * kao;
		//			ray.SetLength(100);
		//			ShapeLayer lineLayer(SpecificGeometryType::LineStringZ);
		//			//lineLayer->SetSourceCrs(utm32n);
		//			lineLayer->SetTargetCrs(eps3794);
		//			ShapeObject name =  lineLayer->CreateShapeObject({{ ray.o.x, ray.o.y, ray.o.z + 100 }, { ray.GetSecondPoint(100).x, ray.GetSecondPoint(100).y, ray.GetSecondPoint(100).z}});					
		//			lineLayer->GetStyle().SetColor({1.0f, 0.0f, 0.0f, 1.0f}).PointSymbol().SetSize(5.0f).GetStyle().OutlineSymbol().SetSize(5.0f);
		//			shapeDrawing->AddShapeLayer(lineLayer);
		//			canvasHelper.ZoomToShapeLayer(lineLayer, false);
		//		}

		//		
		//		auto lasObj = lasContainer->AddLas(file);
		//		SetColorFromScore(Symmerty, lasObj);
		//		bb.ExpandByBoundingBox2d(lasObj->GetBoundingBox2d());
		//		/*auto symb = lasObj->GetStyle().FillSymbol();
		//		symb.SetColor({1.0,0.0f,0.0f,1.0f});*/
		//		//lasObj->SetStyle().FillSymbol().SetColor({1.0,0.0f,0.0f,1.0f});
		//		added.push_back(lasObj);
		//		//applicationInstance->GetWorker().Post([&] { canvasHelper.ZoomToShapeLayer(lineLayer, false); });
		//		
		//	}
		//	canvasHelper.ZoomToBoundingBox(bb);			
		//}
	});
	
	auto ClassifyLiDAR = [&](std::string LasFile) mutable
	{
		Platform::Windows::Console::Console console;
		console->EnableVirtualTerminalSequences(true);

		GF::Extensions::LidarProcessing::GeneralProcessingSettings genericParams;
		genericParams.IsRemoveOutliers = true;
		genericParams.OutliersHeight = 0.1;
		genericParams.OutliersSize = 15;

		// Generate DTM
		APPRESULT_DEV_INFORMATION("Generating DTM");
		GF::Extensions::LidarProcessing::DtmProcessingSettings dtmParams;
		dtmParams.Resolution = 0.5;
		GF::Extensions::LidarProcessing::DtmProcessing dtm(LasFile);
		std::string dtmFile = dtm.Run(genericParams, dtmParams, nullptr);
		
		// Generate PAS
		APPRESULT_DEV_INFORMATION("Generating PAS");
		GF::Extensions::LidarProcessing::PasProcessingSettings pasParams;
		GF::Extensions::LidarProcessing::PasProcessing pas(dtmFile);
		std::string pasFile = pas.Run(pasParams);

		// Classify Ground
		APPRESULT_DEV_INFORMATION("Classifying Ground");
		GF::Extensions::LidarProcessing::GroundClassificationProcessingSettings groundParams; 
		GF::Extensions::LidarProcessing::GroundProcessing ground(LasFile, dtmFile, GF::Extensions::LidarProcessing::PathDepth::OriginalLasFolder);
		std::string groundFile = ground.Run(genericParams, groundParams, nullptr);

		// Classify Buildings
		APPRESULT_DEV_INFORMATION("Classifying Buildings");
		GF::Extensions::LidarProcessing::BuildingClassificationProcessingSettings buildingParams;
		GF::Extensions::LidarProcessing::BuildingsProcessing buildings(groundFile, dtmFile, GF::Extensions::LidarProcessing::PathDepth::GroundFolder);
		std::string buildingsFile = buildings.Run(genericParams, buildingParams, nullptr);

		m_LASFilename = buildingsFile;
		//return;
		// Classify Vegetation
		APPRESULT_DEV_INFORMATION("Classifying Vegetation");
		GF::Extensions::LidarProcessing::VegetationClassificationProcessingSettings vegetationParams;
		vegetationParams.IsSingleClass = false;
		vegetationParams.LowVegetation = 0.2;
		vegetationParams.MiddleVegetation = 1.0;
		vegetationParams.HighVegetation = 3.0;
		GF::Extensions::LidarProcessing::VegetationProcessing vegetation(buildingsFile, dtmFile, GF::Extensions::LidarProcessing::PathDepth::BuildingFolder);
		std::string vegetationFile = vegetation.Run(genericParams, vegetationParams, nullptr);
		//m_LASFilename = vegetationFile;
		
		APPRESULT_DEV_INFORMATION("Classification DONE");
	};

	ribbon->SetOnCommandExecute(ID_CLASSIFY_LIDAR, [&] 
	{
		APPRESULT_DEV_INFORMATION("Classify");

		if (lasContainer->CountLasObjects() > 0)
		{
			auto addedLas = lasContainer->GetLasObject(0);
			ClassifyLiDAR(m_LASFilename);

			shapeDrawing->Clear();
			rasterDrawing->Clear();
			lasContainer->Clear();

			auto lasObj = lasContainer->AddLas(m_LASFilename);
			canvasHelper.ZoomToBoundingBox(lasObj->GetBoundingBox2d());

			RenderWholeLas();
		}
	});
	
	ribbon->SetOnCommandExecute(ID_LIDAR_HOUSES, [&] 
	{
		Platform::Windows::Console::Console console;
		console->EnableVirtualTerminalSequences(true);

		// Get mask at resolution 1
		auto [lasLayer, lasClassMask] = GetClassificationMask(m_LASFilename, 1.0);
		lasClassMask->GetImage()->SaveToFile("ClassMask_Raw.tif");

		// Close small holes in rooftops
		ClosingByReconstruction(lasClassMask, 1);
		lasClassMask->GetImage()->SaveToFile("ClassMask_Closed.tif");

		// Limit classMask to the buildings
		lasClassMask->SetValueLimits(6.0, 6.0);

		console.WriteLine("Extracting geometry bounds from: %s", m_LASFilename);
		// Extract building bounds
		ShapeLayer buildings = GF::Processing::Features::ShapeFeatureExtraction::ExtractPolygonBounds(lasClassMask, 1, false, false, false);
		buildings->SetName("buildings");
		//buildings->InsertHeaderObject("id", DataType::Integer);
		//auto obj = buildings->GetShapeObject(0);
		//obj->SetAttributeValue("id", 33); 
		// Save result
		//GF::Api::Spatial::Shape::ToGeopackage(buildings->AsInterface(), "buildings.gpkg");

		shapeDrawing->Clear();
		rasterDrawing->Clear();
		lasContainer->Clear();
		shapeDrawing->AddShapeLayer(buildings);
		int stop = 1;
	});

	ribbon->SetOnCommandExecute(ID_SYMMETRY_OTHER, [&] {
		APPRESULT_DEV_INFORMATION("Cehi");


	});

	ribbon->SetOnCommandExecute(ID_SYMMETRY_NERAT, [&]
	{
		APPRESULT_DEV_INFORMATION("Nerat");

		auto layer = shapeDrawing->FindLayerByName("buildings");
		if(layer != nullptr)
		{
			auto shapesList = layer->GetSelectedShapes();
			if(shapesList.size() > 0)
			{
				LAS::File lasfile(m_LASFilename);
				std::vector<LAS::Data::Vector3d> list;
				for(size_t i = 0; i < lasfile.size(); i++)
				{
					auto pt = lasfile.TransformCoord(lasfile[i]);
					Geometry::Point3d tmp(pt.x, pt.y, pt.z);

					for(auto shape : shapesList)
					{
						for(auto polygon : shape->GetComposedPolygons())
						{
							if(polygon.Contains2d(tmp))
								list.push_back(pt);
						}
					}
				}
				int stop = 1;
				//TODO: Call Nerat algorithm for symetrie	
			}
		}
	});

	ribbon->SetOnCommandExecute(ID_SYMMETRY_LUKAC, [&] 
	{
		APPRESULT_DEV_INFORMATION("Lukac");

		LAS::File lasfile(m_LASFilename);
		std::vector<LAS::Data::Vector3d> list;
		for (size_t i=0; i < lasfile.size(); i++)
		{
			auto pt = lasfile.TransformCoord(lasfile[i]);
			Geometry::Point2d tmp(pt.x, pt.y);
			if (SelectionBB.Contains(tmp))
			{
				list.push_back(pt);
			}
		}		
		//TODO: call Lukac alg with selected points.
		int stop = 1;
	});

	//
	// Show main window and run event loop
	//
	ribbon->LoadUI();	// Load ribbon UI after everything is set
	mainWindow->Show(SW_SHOW);	// Display window
	mainLayout->Recalculate();
	int returnCode = applicationInstance->Run(0, nullptr); 	// Run main event loop
	shapeLoader->Stop(); 	// Cleanup
	return returnCode;
}
