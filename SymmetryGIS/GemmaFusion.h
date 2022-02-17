#pragma once

// Core
#include <Core/IObject.h>
#include <Core/ArrayView.hpp>
#include <Core/Base.hpp>
#include <Core/CommonBase.hpp>
#include <Core/Exception.hpp>
#include <Core/Helpers.hpp>
#include <Core/Iterator.hpp>
#include <Core/SharedPtr.hpp>
#include <Core/SharedBase.hpp>
#include <Core/StringFormat.hpp>
#include <Core/TypeDefinitions.h>
#include <Core/InternalImplementations.hxx>
#include <Core/magic_enum.hpp>

#include <Geometry/Data.hpp>
#include <Geometry/Geometry.hpp>
#include <Geometry/Wkb.hpp>
#include <Geometry/RTree.hpp>

// Data
#include <Data/Array.hpp>
#include <Data/Collections.hpp>
#include <Data/DateTime.hpp>
#include <Data/JsonDocument.hpp>
#include <Data/JsonObject.hpp>
#include <Data/String.hpp>
#include <Data/Variant.hpp>
#include <Data/Uuid.hpp>
#include <Data/StringConv.hxx>

// Platform
#include <Platform/IO.h>
#include <Platform/Windows/AppLogger.h>
#include <Platform/Windows/AppResult.h>
#include <Platform/Windows/Definitions.h>
#include <Platform/Windows/HttpClient.h>
#include <Platform/Windows/HttpHelpers.h>
#include <Platform/Windows/Process.h>
#include <Platform/Windows/Application.h>
#include <Platform/Windows/Module.h>
#include <Platform/Windows/Windows.h>
#include <Platform/Windows/Threading.h>
#include <Platform/Windows/File.h>
#include <Platform/Windows/PerformanceCounter.hxx>

// TiffTools
#include <TiffTools/Transform.h>
#include <TiffTools/TIFFTools.h>
#include <TiffTools/TIFFTypes.h>

# // UI
#include <Gui/Window.h>
#include <Gui/HwndControl.h>
#include <Gui/BasicControl.h>
#include <Gui/Font.h>
#include <Gui/Bitmap.h>
#include <Gui/DynamicLayout.h>
#include <Gui/TreeView.h>
#include <Gui/ListView.h>

// Graphics
#include <Graphics/GraphicsDevice.h>

// MapServices
#include <MapServices/Wmts/WmtsFetcher.h>
#include <MapServices/Wms/WmsFetcher.h>
#include <MapServices/Wcs/WcsFetcher.h>
#include <MapServices/Tms/TileFetcher.h>
#include <MapServices/Interfaces/ITileFetcher.h>
#include <MapServices/Common/UtmUtils.h>

// STL
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <unordered_map>
#include <cstring>
#include <functional>
#include <optional>
#include <array>
#include <string_view>
#include <exception>
#include <sstream>  
#include <queue>
#include <fstream>
#include <type_traits>
#include <thread>
#include <mutex>
#include <atomic> 

// Concurrency
#include <ppl.h>
#include <ppltasks.h>
#include <concurrent_unordered_map.h>
#include <concurrent_queue.h>

// Gemma Fusion SDK
#include <GF/Core/Interfaces.h>
#include <GF/Core/DataTypes.hpp>
#include <GF/Core/Core.h>
#include <GF/Core/Encoding.hpp>
#include <GF/Core/Encryption.hpp>
#include <GF/Spatial/Geometry.h>
#include <GF/Spatial/Projection/Api.h>
#include <GF/Spatial/Lidar/Api.h>
#include <GF/Spatial/Shape/Api.h>
#include <GF/Spatial/Raster/Api.h>
#include <GF/Core/Core.Private.h>
#include <GF/Data/Data.h>
#include <GF/Database/Database.h>
#include <GF/Spatial/Shape.Private.h>
#include <GF/Spatial/Raster.Private.h>
#include <GF/Spatial/Lidar.Private.h>
#include <GF/Spatial/RasterDrawing.h>
#include <GF/Spatial/ShapeDrawing.h>
#include <GF/Renderers/GraphicsEngine.h>
#include <GF/Renderers/Georeference.h>
#include <GF/Renderers/ShapeRenderer.h>
#include <GF/Renderers/MapLayerRenderer.h>
#include <GF/UI/UI.h>
#include <GF/UI/Components.h>
#include <GF/UI/Mfc/Components.h>
#include <GF/Tasks/Tasks.h>
#include <GF/Extensions/Extensions.h>

// Gemma Fusion private
#include <GF/Core/Core.Private.h>
#include <Private/Settings.h>
#include <Private/GpsSecurity.h>
#include <Private/Spatial/Shape/ShapeLayer.h>
#include <Private/Spatial/Shape/ShapeObject.h>
#include <Private/Spatial/Shape/Routing.h>
#include <Private/Spatial/Shape/Intersection.h>
#include <Private/Spatial/Shape/ExpressionSolver.h>
#include <Private/Spatial/Shape/WKB/WkbHelpers.h>
#include <Private/Spatial/Shape/ShapeUtility.h>
#include <Private/Spatial/Shape/WKT/WKTParser.h>
#include <Private/Spatial/Shape/Backends/Geopackage.h>
#include <Private/Spatial/Shape/Backends/Shapefile.h>
#include <Private/Spatial/Shape/Backends/GeoJson.h>
#include <Private/Spatial/Raster/Raster.h>
#include <Private/Spatial/Raster/RasterLayer.h>
#include <Private/Spatial/Raster/RasterObject.h>
#include <Private/Spatial/Lidar/LidarManager.h>
#include <Private/Spatial/Lidar/LidarClassification.h>
#include <Private/Data/ColorMap.h>
#include <Private/Data/SQLite.h>
#include <Private/Data/Table.h>
#include <LAS\File.h>


// 3rd party
#include <pqxx/result_iterator.hxx>
#define NANODBC_ENABLE_UNICODE
#include <nanodbc/nanodbc.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/filereadstream.h>

namespace GemmaFusion {
	using namespace Platform::Windows::Application;
	using namespace Platform::Windows::Module;
	using namespace Platform::Windows::Threading;
	using namespace Platform::Windows::Logger;
	using namespace Platform::Windows::Network::Http;
	using namespace UI;
	using namespace Tasks;

	using namespace MapServices;
	using namespace MapServices::Tms;
	using namespace MapServices::Wms;
	using namespace MapServices::Wmts;

	using namespace GF::Common;
	using namespace GF::Data;
	using namespace GF::Database;
	using namespace GF::Api::Spatial::Projection;
	using namespace GF::Spatial::Shape;
	using namespace GF::Spatial::Raster;
	using namespace GF::Spatial::Lidar;
	using namespace GF::Spatial::Drawing;
	using namespace GF::UI::Components;
	using namespace GF::Renderers::Maps;
	using namespace GF::Renderers::Shape;
	using namespace GF::Renderers::Raster;
	using namespace Geometry;
	using namespace std::string_literals;
	using namespace std::string_view_literals;

	template<typename T>
	using Ptr = Core::Ptr<T>;
	using GraphicsDevice9 = GF::Renderers::GraphicsEngine::GraphicsDevice;
	using Camera = GF::Renderers::GraphicsEngine::Camera;
	using Mouse = GF::Renderers::GraphicsEngine::Mouse;
	using Keyboard = GF::Renderers::GraphicsEngine::Keyboard;
	using WorldGeoreferencer = GF::Renderers::Georeference::WorldGeoreferencer;
	using ShapeRenderer = GF::Renderers::Shape::MonoLayerRenderer;

	using GraphicsInfrastructure = ::Graphics::GraphicsInfrastructure;
	using GraphicsDevice11 = ::Graphics::GraphicsDevice;
}